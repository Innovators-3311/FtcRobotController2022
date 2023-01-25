package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.PIDController;
import java.util.Locale;

@Autonomous(name="MezDoom", group="Exercises")
public class MezImu  extends LinearOpMode
{


    private MecanumDriveBase mecanumDriveBase;
    private ElapsedTime elapsedTime;

    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorCenter;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, strafeCorrection,  rotation;

    double distance;

    PIDController pidRotate, pidDrive, pidStrafe;

    // called when init button is  pressed.

    private void initialize()
    {
        initImu();

        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
        elapsedTime = new ElapsedTime();

        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorCenter = hardwareMap.get(DistanceSensor.class, "distanceSensorCenter");

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);
        telemetry.addData("Hit start", "");
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException
    {

        initialize();
        int round = 1;
        // Wait until we're told to go
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Round " + round, "");
            telemetry.update();
            basicRotate(360, 0.2, true);
            telemetry.addData("done", "");
            telemetry.update();
            sleep(1500);
        }
        sleep(50000);
        bullDogAttack();


//        findPole();

//        while(opModeIsActive()) {
//            double distanceL = distanceSensorLeft.getDistance(DistanceUnit.INCH);
//            double distanceR = distanceSensorRight.getDistance(DistanceUnit.INCH);
//            double distanceC = distanceSensorCenter.getDistance(DistanceUnit.INCH);
//            telemetry.addData("correction", correction);
//            telemetry.addData("distanceL", distanceL);
//            telemetry.addData("distanceR", distanceR);
//            telemetry.addData("distanceC", distanceC);
//            telemetry.update();
//            sleep(1000);
//        }

        rotate(90,  0.3);
        sleep(1000000);

        while(opModeIsActive())
        {
            double correction = AlignRobot();

            double distanceL = distanceSensorLeft.getDistance(DistanceUnit.INCH);
            double distanceR = distanceSensorRight.getDistance(DistanceUnit.INCH);
            double distanceC = distanceSensorCenter.getDistance(DistanceUnit.INCH);
            telemetry.addData("correction", correction);
            telemetry.addData("distanceL", distanceL);
            telemetry.addData("distanceR", distanceR);
            telemetry.addData("distanceC", distanceC);
            telemetry.update();




            // Both see it positive turn right
            // Both see it negative turn left
            sleep(500);
            if (distanceSensorCenter.getDistance(DistanceUnit.INCH) < 24)
            {
                while (distanceSensorCenter.getDistance(DistanceUnit.INCH) > 8 && distanceSensorRight.getDistance(DistanceUnit.INCH) > 5 && distanceSensorLeft.getDistance(DistanceUnit.INCH) > 5)
                {
                    mecanumDriveBase.driveMotors(0.2, 0, 0, 1);
                    telemetry.addData("distanceC ", distanceSensorCenter.getDistance(DistanceUnit.INCH));
                    telemetry.addData("distanceR ", distanceSensorRight.getDistance(DistanceUnit.INCH));
                    telemetry.addData("distanceL ", distanceSensorLeft.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                mecanumDriveBase.driveMotors(0, 0, 0, 0);
            }
//            else
//            {
//                findPole(18);
//            }

            correction = AlignRobot();

//            if (Math.abs(correction) < 1)
//            {
//                mecanumDriveBase.driveMotors(0, 0, 0, 0);
//            }

            sleep(500);
            if (correction == -1000)  //right see it, left does not
            {
                while (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 24)
                {
                    //Turn right until left sensor see it.
                    mecanumDriveBase.driveMotors(0, -.3, 0, 1);
                    telemetry.addData("distanceL ", distanceSensorLeft.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                mecanumDriveBase.driveMotors(0, 0, 0, 0);
            }
            else if (correction == 1000) // left see it, left does not
            {
                while (distanceSensorRight.getDistance(DistanceUnit.INCH) > 24)
                {
                    // turn left untill sensor see it
                    mecanumDriveBase.driveMotors(0, .3, 0, 1);
                    telemetry.addData("distanceR ", distanceSensorRight.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                mecanumDriveBase.driveMotors(0, 0, 0, 0);
            }

            correction = AlignRobot();

//            else
//            {
//                correction = correction / 10;
////                mecanumDriveBase.driveMotors(0, correction, 0, 1);
//            }
            sleep(500);

            if (correction > 0.25 && correction < 100)
            {
                while (correction >= 0.25 && distanceSensorCenter.getDistance(DistanceUnit.INCH)  > 24)
                {
                    correction = AlignRobot();
                    mecanumDriveBase.driveMotors(0, 0.3, 0, 1);
                }
                mecanumDriveBase.driveMotors(0, 0, 0, 0);
            }
            else if (correction < 0.25 && correction > 100)
            {
                while (correction <= 0.25 && distanceSensorCenter.getDistance(DistanceUnit.INCH)  > 24)
                {
                    correction = AlignRobot();
                    mecanumDriveBase.driveMotors(0, -0.3, 0, 1);
                }
                mecanumDriveBase.driveMotors(0, 0, 0, 0);
            }

            correction = AlignRobot();

            if (distanceSensorCenter.getDistance(DistanceUnit.INCH) < 24)
            {
                while (distanceSensorCenter.getDistance(DistanceUnit.INCH) > 4)
                {
                    mecanumDriveBase.driveMotors(0.2, 0, 0, 1);
                    telemetry.addData("distanceC ", distanceSensorCenter.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                mecanumDriveBase.driveMotors(0, 0, 0, 0);
            }

            telemetry.addData("Done", "");
            telemetry.update();
            sleep(1000);

        }

        // Start the logging of measured acceleration
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard

//            rotate(-120,  1);

//            telemetry.update();

//            sleep(1000);

//            rotate(120,  1);

//            telemetry.update();

//            sleep(1000);

//            closeDistanceRight(30);

        stop();
    }

    private void initImu() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private double rotate(int degrees, double power) {
        // restart imu angle tracking.
        telemetry.addData("Rotating", "");
        telemetry.update();
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                //leftMotor.setPower(power);
                //rightMotor.setPower(-power);

                mecanumDriveBase.driveMotors(0, power, 0, 1);

                telemetry.addData("rotate off zero", "angle: " + getAngle() + "degrees:" + degrees);
                telemetry.update();

                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                //leftMotor.setPower(-power);
                //rightMotor.setPower(power);

                telemetry.addData("rotate2", "angle2: " + getAngle() + "degrees:" + degrees);
                telemetry.addData("rotate2", "power: " + power +" setpoint: " + pidRotate.getSetpoint());
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.update();

                mecanumDriveBase.driveMotors(0, -power, 0, 1);

                distance = checkDistance(0.0, 80.0);
                if (distance != -1) {
                    //WE SEE SOMETHING IN THE GIVEN RANGE.  STOP NOW!!!!!!
                    //    pidRotate.setSetpoint(pidRotate.getSetpoint());
                    //need to make this case enter only once
                    break;
                }

            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                //leftMotor.setPower(-power);
                //rightMotor.setPower(power);

                distance = checkDistance(0.0, 15.0);
                if (distance != -1)
                {
                    /*
                    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    while (checkDistance(0.0, 80.0) < 30)
                    {

                        //do nothing
                    }
                    mecanumDriveBase.driveMotors(0, -power, 0, 1);
                    Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    double deltaAngle = angles2.firstAngle - angles.firstAngle;
*/


                    //WE SEE SOMETHING IN THE GIVEN RANGE.  STOP NOW!!!!!!
                    //    pidRotate.setSetpoint(pidRotate.getSetpoint());
                    //need to make this case enter only once
                    break;
                }


                mecanumDriveBase.driveMotors(0, -power, 0, 1);

            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        //rightMotor.setPower(0);
        //leftMotor.setPower(0);
        mecanumDriveBase.driveMotors(0, 0, 0, 1);

        telemetry.addData("distance", distance);
        telemetry.update();

//        sleep(10000);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();

        //TODO: return the number of degrees it did turn.  (in case distance preempted turn)
        return 0.0;
    }



    /**
     * Check to see if the sensor detects an object within the range provided.
     * @param innerBound minimum range detection.
     * @param outerBound maximum range of detection.
     */
    private double checkDistance(double innerBound, double outerBound)
    {
        double distance = distanceSensorCenter.getDistance(DistanceUnit.INCH);

        if (distance < outerBound && distance > innerBound)
        {
            distance = distanceSensorCenter.getDistance(DistanceUnit.INCH);
        }
        else
        {
            distance = -1;
        }

        return distance;
    }

    private double AlignRobot()
    {
        double xCorrection = 0.0;
        if (distanceSensorRight.getDistance(DistanceUnit.INCH) <= 30 && distanceSensorLeft.getDistance(DistanceUnit.INCH) <= 30)
        {
            xCorrection = (-distanceSensorRight.getDistance(DistanceUnit.INCH) + distanceSensorLeft.getDistance(DistanceUnit.INCH));

            //Pole is detected by both sensors.
            //Negative value is Left leaning location.  Positive value is Right leaning value.
            return xCorrection;
        }
        else if (distanceSensorRight.getDistance(DistanceUnit.INCH) > 30 && distanceSensorLeft.getDistance(DistanceUnit.INCH) <= 30)
        {
            //Pole is detected by Left sensor only
            return 1000;
        }
        else if (distanceSensorRight.getDistance(DistanceUnit.INCH) <= 30 && distanceSensorLeft.getDistance(DistanceUnit.INCH) > 30)
        {
            //Pole is detected by Right sensor only
            return -1000;
        }
        else
        {
            //Does not see anything
            return -1;
        }

    }

    private void bullDogAttack()
    {
        double distanceL, distanceR, distanceC = 0.0;

        double drivePower = 0.0;

        //normally would continue until button is unpressed, but have no controller
        for (int i = 0; i < 1000; i++)
        {
            double correction = AlignRobot();

            distanceC = distanceSensorCenter.getDistance(DistanceUnit.INCH);
            distanceR = distanceSensorRight.getDistance(DistanceUnit.INCH);
            distanceL = distanceSensorLeft.getDistance(DistanceUnit.INCH);

            telemetry.addData("distanceC ", distanceC);
            telemetry.addData("distanceR ", distanceR);
            telemetry.addData("distanceL ", distanceL);
            telemetry.addData("correction ", correction);
            //telemetry.addData("correction/50" , correction / 50);
            telemetry.update();

            if (distanceC > 100)
            {
                drivePower = 0;
            }
            else if (distanceC < 30 && distanceC > 8)
            {
                drivePower = 0.2;
            }
            else if (distanceC < 8)
            {
                drivePower = -0.2;
            }
            //drivePower = 0;

            if (Math.abs(correction) < 1)
            {
                mecanumDriveBase.driveMotors(drivePower, 0, 0, 0);
            }
            else if (correction == -1000)  //right see it, left does not
            {
                //Turn right until left sensor see it.
                mecanumDriveBase.driveMotors(0, .2, 0, 1);
            }
            else if (correction == 1000) // left see it, left does not
            {
                // turn left untill sensor see it
                mecanumDriveBase.driveMotors(0, -.2, 0, 1);
            }
            else
            {
                correction = correction / 10;
                if (correction < -.1)
                mecanumDriveBase.driveMotors(drivePower, -0.2, 0, 1);
                else if (correction > 0.1)
                    mecanumDriveBase.driveMotors(drivePower, 0.2, 0, 1);
                else
                    mecanumDriveBase.driveMotors(drivePower, correction, 0, 1);

            }

        }
    }

    private void findPole(double range)
    {
        turnRightTillPole(range);
        if (distanceSensorCenter.getDistance(DistanceUnit.INCH) > range)
        {
            turnLeftTillPole(range);
        }
    }

    private void turnRightTillPole(double range)
    {
        elapsedTime.reset();
        elapsedTime.startTime();
        while (distanceSensorCenter.getDistance(DistanceUnit.INCH) > range && elapsedTime.seconds() < 3)
        {
            distance = distanceSensorCenter.getDistance(DistanceUnit.INCH);
            telemetry.addData("distance", distance);
            telemetry.update();


            mecanumDriveBase.driveMotors(0, 0.2, 0, 1);
        }

        mecanumDriveBase.driveMotors(0, 0, 0, 0);
    }

    private void turnLeftTillPole(double range)
    {
        elapsedTime.reset();
        elapsedTime.startTime();

        while (distanceSensorCenter.getDistance(DistanceUnit.INCH) > range && elapsedTime.seconds() < 6) {
            distance = distanceSensorCenter.getDistance(DistanceUnit.INCH);
            telemetry.addData("distance", distance);
            telemetry.update();

            mecanumDriveBase.driveMotors(0, -0.2, 0, 1);
        }

        mecanumDriveBase.driveMotors(0, 0, 0, 0);

    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void basicRotate(double degrees, double power, boolean sensor)
    {
        double powerRate = 0;
        double distance;
        double firstPole = 0;
        double secondPole  = 0;
        boolean recenterOnPole = false;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            telemetry.addData("Turn right", "");
            telemetry.update();
            powerRate = power;
        }
        else if (degrees > 0)
        {   // turn left.
            telemetry.addData("Turn left", "");
            telemetry.update();
            powerRate = -power;
        }
        else return;

        // set power to rotate.
        mecanumDriveBase.driveMotors(0, powerRate, 0, 1);

        boolean foundPole = false;

        // rotate until turn is completed.
        if (!sensor)
        {
            telemetry.addData("Simple turn", "");
            telemetry.update();
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive())
            {
                if (opModeIsActive() && Math.abs(getAngle()) > Math.abs(degrees))
                {
                    mecanumDriveBase.driveMotors(0, 0, 0, 0);
                    telemetry.addData("Stop", "");
                    telemetry.update();
                    break;
                }
            }
        }
        else
        {
            // left turn.
            while (opModeIsActive() && Math.abs(getAngle()) < Math.abs(degrees))
            {
                if (sensor)
                {
                    telemetry.addData("Sensor in use","");
                    telemetry.update();
                    distance = checkDistance(0, 24);
                    if ((distance != -1) && !foundPole)
                    {
                        telemetry.addData("found pole", "");
                        telemetry.update();
                        firstPole = getAngle();
                        foundPole = true;
                    }

                    distance = checkDistance(0, 24);
                    if (foundPole && distance == -1)
                    {
                        telemetry.addData("lost pole", "");
                        telemetry.update();
                        mecanumDriveBase.driveMotors(0, 0, 0, 0);
                        secondPole = getAngle();
                        recenterOnPole = true;
                        break;
                    }
                }
            }
        }

        // turn the motors off.
        mecanumDriveBase.driveMotors(0, 0, 0, 0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();

        //turn back in the reverse direction to center on the scanned pole.
        if (recenterOnPole && sensor)
        {
            telemetry.addData("recentering on pole", "");
            telemetry.update();

            if (degrees > 0)
            {
                double angleCorrection = (firstPole - secondPole) / 2;
                basicRotate((int) angleCorrection, 0.2, false);
            }
            else
            {
                double angleCorrection = (firstPole - secondPole) / 2;
                basicRotate((int) angleCorrection, 0.2, false);
            }

        }
    }

}
