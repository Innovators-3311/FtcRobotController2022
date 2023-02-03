package org.firstinspires.ftc.teamcode.util.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.ConeDetection;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.TeamDetection;

import java.util.Locale;

@Autonomous(name="AutonomousHigh", group="Exercises")
public class WapaAutoHigh extends LinearOpMode
{
    private CameraInitSingleton cameraInitSingleton;
   // private WebcamName webcam;
    private ConeDetection coneDetection;
    private MecanumDriveBase mecanumDriveBase;
    private ElapsedTime elapsedTime;
    private TeamDetection teamDetection;

    private TouchSensor lowSensor;

    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorCenter;

    private DcMotor screw;
    private DcMotor uBar;
    private DcMotor intake;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double  globalAngle;
    double  correction;
    double  rotation;
    double  initAngle;

    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    double distance;
    double toPole = 0;
    double leftFrontPos;
    double RightFrontPos;
    double LeftBackPos;

    boolean blueTeam = true;
    int zone = -1;

    PIDController pidRotate, pidDrive, pidStrafe;

    // called when init button is  pressed.

    private void initialize()
    {
        initImu();

        cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        teamDetection = new TeamDetection(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        coneDetection = new ConeDetection(hardwareMap, cameraInitSingleton.getWebcam());
        elapsedTime = new ElapsedTime();

        distanceSensorCenter = hardwareMap.get(DistanceSensor.class, "distanceSensorCenter");

        //Map Hardware
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");

        screw.setDirection(DcMotor.Direction.REVERSE);
        uBar.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);


        initAngle = getHeading();
//        RobotLog.ii("WAPA initAngle:", "%d", initAngle);

        blueTeam = teamDetection.showTeam(telemetry);

        //Code to prevent my fingers from bleeding...  (manual lowering of screw)
//        while(!lowSensor.isPressed())
//        {
//            screw.setPower(-0.5);
//        }
//        screw.setPower(0);
//        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sleep(10000);
//        telemetry.addData("Setting Ubar in 10 seconds", "");
//        telemetry.update();
//        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Hit start", "");
    }

    /**
     * Init the IMU code
     */
    private void initImu()
    {
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



    @Override
    public void runOpMode() throws InterruptedException
    {
        //Initialize
        initialize();

        //Detecting the code before waitForStart may have been causing us issues.  At any point it
        //is not needed here, and the init preview still works and will ID the code that it sees.
        RobotLog.ii("WAPA Auto", "Waiting for start..");

        telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
        telemetry.addData("", "Terminal = " + blueTeam);
        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        zone = coneDetection.detector(telemetry);

        telemetry.addData("Detected zone", zone);
        RobotLog.ii("WAPA Detected zone:", "%d", zone);
        telemetry.update();
//        sleep(3000);
        //If the detection failed, then select a random zone to park in.  1/3 change of getting it
        //correct
        if (zone == -1)
        {
            if (blueTeam)
            {
                zone = 1;
            }
            else
            {
                zone = 3;
            }
        }

        //Drive forward 62 inches
        driveStraight(ticksPerInch * 62, 1, 0.3);

        //Raise screw and ubar to reach high pole
        sleep(250);
        driveScrew(3300);
        sleep(4000);
        driveUBar(-1800);
        //driveStraight(ticksPerInch * 6, -1, 0.3);

        //Back up from moving the cone.
        driveStraight(ticksPerInch * 8, -1, 0.3);
        sleep(250);
        if (blueTeam)
        {
            //rotate to get near pole, then start a sweep for the pole
            basicRotate(-75, 0.5, false);
            sleep(250);
            basicRotate(-120, 0.3, true);
        }
        else
        {
            //rotate to get near pole, then start a sweep for the pole
            basicRotate(75, 0.5, false);
            sleep(250);
            basicRotate(120, 0.3, true);
        }

        //If we still see the pole after rotation...
        if (distanceSensorCenter.getDistance(DistanceUnit.INCH) < 24)
        {
            toPole = distanceSensorCenter.getDistance(DistanceUnit.INCH) - 5;

            telemetry.addData("Pole", toPole);
            RobotLog.ii("WAPA distance to move to pole:", "%f", toPole);

            driveStraight(ticksPerInch * toPole, -1, 0.2);
            sleep(1000);
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
            //driveStraight(ticksPerInch * (toPole + 5), 1, 0.3);
            driveStraight(ticksPerInch * (toPole + 3.5), 1, 0.3);
        }

        //Need move to park zone.  Turn to 90 degree angle of start direction.  Find current
        //heading and turn robot to face 90 or -90.
        double ang = getAngle();

        if ((zone == 1) || (zone == 3))
        {
            if (blueTeam)
            {
                //angles.firstAngle;
                double ang2 = 90 - angles.firstAngle + initAngle;
                double ang3 = angleToHeading(90);
                telemetry.addData("WapaAuto", "heading = " + angles.firstAngle + "ang2 = " + ang2 + " ang3: " + ang3);
                RobotLog.ii("WAPA :", " heading %f ang2 = %f  ang3 = %f", angles.firstAngle, ang2, ang3);
                RobotLog.ii("WAPA :", " ang3 %f", ang3);
                telemetry.update();
                sleep(250);
                basicRotate(ang2, 0.5, false);
                if (zone == 1)
                {
                    driveStraight(ticksPerInch * 21, -1, 0.5);
                    basicRotate(-90, 0.5, false);
                }
                if (zone == 3)
                {
                    driveStraight(ticksPerInch * 17, 1, 0.5);

                }
            }
            else
            {
                double ang2 = 90 - angles.firstAngle + initAngle;
                double ang3 = angleToHeading(90);
                telemetry.addData("WapaAuto", "heading = " + angles.firstAngle + "ang2 = " + ang2 + " ang3: " + ang3);
                RobotLog.ii("WAPA :", " heading %f ang2 = %f  ang3 = %f", angles.firstAngle, ang2, ang3);
                RobotLog.ii("WAPA :", " ang3 %f", ang3);
                telemetry.update();
                sleep(250);
                basicRotate(ang2, 0.5, false);
                if (zone == 1)
                {
                    driveStraight(ticksPerInch * 17, -1, 0.5);
                }
                if (zone == 3)
                {
                    driveStraight(ticksPerInch * 22, 1, 0.5);
                    basicRotate(-90, 0.5, false);
                    driveStraight(ticksPerInch * 2, 1, 0.5);
                }
            }
        }
        else if (zone == 2)
        {
            //TODO: need correct calculation here
            double currHeading = checkOrientation();
            double ang2 = 0 + angles.firstAngle + initAngle;
            double ang3 = angleToHeading(0);
            telemetry.addData("WapaAuto", "heading = " + angles.firstAngle + "ang2 = " + ang2 + " ang3: " + ang3) ;
            telemetry.update();
            RobotLog.ii("WAPA :", " heading %f ang2 = %f  ang3 = %f  initAngle %f  currHeading = %f", angles.firstAngle, ang2, ang3, initAngle, currHeading);
            sleep(250);
            basicRotate(-ang2, 0.5, false);

            currHeading = checkOrientation();
            RobotLog.ii("WAPA :", " currHeading = %f", currHeading);

        }

//        driveScrew(50);
 //       RobotLog.ii("WAPA screw:", "%f", screw.getCurrentPosition());
        //give time for the screw to get to location before stop()
        driveUBar(-1600);
        driveScrew(3400);
        sleep(3000);

        //zero position for tele-op.
        stop();
    }

    private void mediumPoleRun()
    {

        driveStraight(ticksPerInch * 2, 1, 0.3);
        double heading = getHeading();
        telemetry.addData("heading", heading);
        telemetry.update();
//        sleep(5000);
//        basicRotate(-heading,0.3,false);
        driveUBar(-1800);
        //Drive Forward
        driveStraight(ticksPerInch * 49, 1, 0.3);
        sleep(100);
        driveStraight(ticksPerInch * 3, -1, 0.3);

//        driveScrew(100);


        sleep(100);

        if (blueTeam)
        {
            basicRotate(-120, 0.3, true);
        }
        else
        {
            basicRotate(120, 0.3, true);
        }

        if (distanceSensorCenter.getDistance(DistanceUnit.INCH) < 24)
        {
            //TODO: sensor moved.  Need to check this value again
            toPole = distanceSensorCenter.getDistance(DistanceUnit.INCH) - 4;

            telemetry.addData("Pole", toPole + " raw: " + distanceSensorCenter.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(200);
            RobotLog.ii("WAPA Pole:", "%f", toPole);

            driveStraight(ticksPerInch * toPole, -1, 0.2);
            sleep(1000);
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
            driveStraight(ticksPerInch * (toPole), 1, 0.3);
        }

        if ((zone == 1) || (zone == 3))
        {
            //angles.firstAngle;
            //double ang2 = getHeading() - (90 + (180 - Math.abs(initAngle)));
            double ang2 = -90 - getHeading();
            RobotLog.ii("WAPA Turn Angle:", "%f  Curr Heading %f", ang2,angles.firstAngle);
            telemetry.addData("WapaAuto", "heading = " + angles.firstAngle + "ang2 = " + ang2) ;
            telemetry.update();
            sleep(250);
            basicRotate(ang2, 0.5, false);
            if (zone ==  1)
            {
                driveScrew(1);
                driveStraight(ticksPerInch * 24, -1, 0.5);
                //double ang2 = getHeading() - (90 + (180 - Math.abs(initAngle)));
                double ang3 = -90 - (getHeading() - 90);
                RobotLog.ii("WAPA Turn Angle:", "%f  Curr Heading %f", ang3,getHeading());
                basicRotate(ang3, 0.5, false);
            }
            if (zone ==  3)
            {
                driveScrew(1);
                driveStraight(ticksPerInch * 16, 1, 0.5);
            }
        }
        else if (zone == 2)
        {
            //TODO: need correct calculation here
            double ang2 = 0 + angles.firstAngle;
            telemetry.addData("WapaAuto", "heading = " + angles.firstAngle + "ang2 = " + ang2) ;
            telemetry.update();
            sleep(5000);
            basicRotate(ang2, 0.5, false);
        }

    }

    private void driveScrew(int target)
    {
        screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        screw.setTargetPosition(target);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        screw.setPower(1);
    }

    private void driveUBar(int target)
    {
        uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uBar.setTargetPosition(target);
        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uBar.setPower(1);
    }

    private void driveStraight(double target, int forward, double speed)
    {
        speed *= forward;
        leftFrontPos = mecanumDriveBase.lf.getCurrentPosition();
        RightFrontPos = mecanumDriveBase.rf.getCurrentPosition();
        LeftBackPos = mecanumDriveBase.lb.getCurrentPosition();

        RobotLog.ii("WAPA od wheels:", "LF %f RF %f LB: %f", LeftBackPos, RightFrontPos,LeftBackPos);

        // Use gyro to drive in a straight line.
        correction = checkDirection();


        if (forward == 1)
        {
            leftFrontPos += target;
            while (mecanumDriveBase.lf.getCurrentPosition() <= leftFrontPos)
            {
                mecanumDriveBase.driveMotors(speed, -.02, 0, 1);
                telemetry.addData("", mecanumDriveBase.lf.getCurrentPosition());
//                telemetry.update();
            }
        }
        else
        {
            leftFrontPos -= target;
            while (mecanumDriveBase.lf.getCurrentPosition() >= leftFrontPos)
            {
                //TODO: do we need turn value here as well?
                mecanumDriveBase.driveMotors(speed, 0.1, 0, 1);
                telemetry.addData("", mecanumDriveBase.lf.getCurrentPosition());
//                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     * @param power Power setting from 0 to 1
     * @param sensor Boolean indicating if to stop on center sensor detection
     */
    private void basicRotate(double degrees, double power, boolean sensor)
    {
        double powerRate = 0;
        double distance = 0;
        double firstPole = 0;
        double secondPole  = 0;
        boolean recenterOnPole = false;
        boolean foundPole = false;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
//            telemetry.addData("Turn right", "");
//            telemetry.update();
            powerRate = power;
        }
        else if (degrees > 0)
        {   // turn left.
//            telemetry.addData("Turn left", "");
//            telemetry.update();
            powerRate = -power;
        }
        else return; //angle is 0

        // set power to rotate.
        mecanumDriveBase.driveMotors(0, powerRate, 0, 1);


        // rotate until turn is completed.
        if (!sensor)
        {
//            telemetry.addData("Simple turn", "");
//            telemetry.update();
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive())
            {
                if (opModeIsActive() && Math.abs(getAngle()) > Math.abs(degrees))
                {
                    mecanumDriveBase.driveMotors(0, 0, 0, 0);
//                    telemetry.addData("Stop", "");
//                    telemetry.update();
                    break;
                }
            }
        }
        else           //Following code only used if sensor is in play
        {
            // left turn.
            while (opModeIsActive() && Math.abs(getAngle()) < Math.abs(degrees))
            {
                if (sensor)
                {
//                    telemetry.addData("Sensor in use","");
//                    telemetry.update();
                    distance = checkDistance(0, 20);
                    if ((distance != -1) && !foundPole)
                    {
                        //telemetry.addData("found pole", "");
                        //telemetry.update();
                        firstPole = getAngle();
                        telemetry.addData("Angle #1::", firstPole + "Dis: " + distance);
                        RobotLog.ii("WAPA First Angle:", "%f  dis: %f", firstPole, distance);
                        foundPole = true;
                    }

                    distance = checkDistance(0, 20);
                    if (foundPole && distance == -1)
                    {
                        //telemetry.addData("lost pole", "");
                        //telemetry.update();
                        mecanumDriveBase.driveMotors(0, 0, 0, 0);
                        sleep(100);
                        secondPole = getAngle();
                        telemetry.addData("Angle #2:", secondPole + "DIS:" + distance);
                        RobotLog.ii("WAPA Second Pole:", "%f dis %f", secondPole,distance);
                        recenterOnPole = true;
                        break;
                    }
                }
            }
        }

        // turn the motors off.
        mecanumDriveBase.driveMotors(0, 0, 0, 0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();

        //turn back in the reverse direction to center on the scanned pole.
        if (recenterOnPole && sensor)
        {
            //telemetry.addData("recentering on pole", "");
            //telemetry.update();

            if (degrees > 0)
            {
                double angleCorrection = (firstPole - secondPole) / 2;
                basicRotate(angleCorrection, 0.3, false);
                int temp = (int)angleCorrection;
                telemetry.addData("Angle Correction:", angleCorrection + " int: " + temp);
                RobotLog.ii("WAPA Angle Correction:", "%f ", angleCorrection);
            }
            else
            {
                double angleCorrection = (firstPole - secondPole) / 2;
                basicRotate(angleCorrection, 0.3, false);
                int temp = (int)angleCorrection;
                telemetry.addData("Angle Correction:", angleCorrection + " int: " + temp);
                RobotLog.ii("WAPA Angle Correction:", "%f ", angleCorrection);
            }
            telemetry.update();
        }
    }

    private void testScrew()
    {

        /*
        while (opModeIsActive())
        {
            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
            telemetry.addData("Screw Target: ", screw.getTargetPosition());

            telemetry.addData("Mez heading: ", getHeading());
            telemetry.update();
        }
*/

//        while (opModeIsActive())
        {
            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
            telemetry.update();
            driveScrew(2000);
            sleep(5000);
            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
            telemetry.update();
            driveScrew(1);
            sleep(5000);
            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
            telemetry.update();
        }

    }

    double getHeading()
    {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading =  angles.firstAngle;

        if (heading < 0)
        {
            heading = heading + 180;
        }
        else
        {
           heading = heading - 180;
        }

        RobotLog.ii("WAPA Init Heading:", "%f ", heading);

        return heading;
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

        //        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })

//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });

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

//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel * gravity.xAccel
//                                        + gravity.yAccel * gravity.yAccel
//                                        + gravity.zAccel * gravity.zAccel));
//                    }
//                });
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




//    private double rotate(int degrees, double power) {
//        // restart imu angle tracking.
//        telemetry.addData("Rotating", "");
//        telemetry.update();
//        resetAngle();
//
//        // if degrees > 359 we cap at 359 with same sign as original degrees.
//        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
//
//        // start pid controller. PID controller will monitor the turn angle with respect to the
//        // target angle and reduce power as we approach the target angle. This is to prevent the
//        // robots momentum from overshooting the turn after we turn off the power. The PID controller
//        // reports onTarget() = true when the difference between turn angle and target angle is within
//        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
//        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
//        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
//        // turning the robot back toward the setpoint value.
//
//        pidRotate.reset();
//        pidRotate.setSetpoint(degrees);
//        pidRotate.setInputRange(0, degrees);
//        pidRotate.setOutputRange(0, power);
//        pidRotate.setTolerance(1);
//        pidRotate.enable();
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        // rotate until turn is completed.
//
//        if (degrees < 0) {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && getAngle() == 0) {
//                //leftMotor.setPower(power);
//                //rightMotor.setPower(-power);
//
//                mecanumDriveBase.driveMotors(0, power, 0, 1);
//
//                telemetry.addData("rotate off zero", "angle: " + getAngle() + "degrees:" + degrees);
//                telemetry.update();
//
//                sleep(100);
//            }
//
//            do {
//                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
//                //leftMotor.setPower(-power);
//                //rightMotor.setPower(power);
//
//                telemetry.addData("rotate2", "angle2: " + getAngle() + "degrees:" + degrees);
//                telemetry.addData("rotate2", "power: " + power +" setpoint: " + pidRotate.getSetpoint());
//                telemetry.addData("1 imu heading", lastAngles.firstAngle);
//                telemetry.addData("2 global heading", globalAngle);
//                telemetry.addData("3 correction", correction);
//                telemetry.addData("4 turn rotation", rotation);
//                telemetry.update();
//
//                mecanumDriveBase.driveMotors(0, -power, 0, 1);
//
//                distance = checkDistance(0.0, 80.0);
//                if (distance != -1) {
//                    //WE SEE SOMETHING IN THE GIVEN RANGE.  STOP NOW!!!!!!
//                    //    pidRotate.setSetpoint(pidRotate.getSetpoint());
//                    //need to make this case enter only once
//                    break;
//                }
//
//            } while (opModeIsActive() && !pidRotate.onTarget());
//        } else    // left turn.
//            do {
//                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
//                //leftMotor.setPower(-power);
//                //rightMotor.setPower(power);
//
//                distance = checkDistance(0.0, 15.0);
//                if (distance != -1)
//                {
//                    /*
//                    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                    while (checkDistance(0.0, 80.0) < 30)
//                    {
//
//                        //do nothing
//                    }
//                    mecanumDriveBase.driveMotors(0, -power, 0, 1);
//                    Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                    double deltaAngle = angles2.firstAngle - angles.firstAngle;
//*/
//
//
//                    //WE SEE SOMETHING IN THE GIVEN RANGE.  STOP NOW!!!!!!
//                    //    pidRotate.setSetpoint(pidRotate.getSetpoint());
//                    //need to make this case enter only once
//                    break;
//                }
//
//
//                mecanumDriveBase.driveMotors(0, -power, 0, 1);
//
//            } while (opModeIsActive() && !pidRotate.onTarget());
//
//        // turn the motors off.
//        //rightMotor.setPower(0);
//        //leftMotor.setPower(0);
//        mecanumDriveBase.driveMotors(0, 0, 0, 1);
//
//        telemetry.addData("distance", distance);
//        telemetry.update();
//
////        sleep(10000);
//
//        rotation = getAngle();
//
//        // wait for rotation to stop.
//        sleep(500);
//
//        // reset angle tracking on new heading.
//        resetAngle();
//
//        //TODO: return the number of degrees it did turn.  (in case distance preempted turn)
//        return 0.0;
//    }


    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        RobotLog.ii("WAPA Correction:", "" + correction);
        return correction;
    }

    private double angleToHeading(double destination)
    {
        //current heading
        double origin = getAngle();
        double rightAngle = 0.0;
        double leftAngle = 0.0;

        rightAngle = (destination - origin);
        if (rightAngle < 0)
        {
            rightAngle = rightAngle + 360;
        }

        leftAngle = (origin - destination);
        if (leftAngle < 0)
        {
            leftAngle = leftAngle + 360;
        }

        RobotLog.ii("WAPA angleToHeading:", "org: " + origin + " des: " + destination + " right: " + rightAngle + " left: " + leftAngle);
        //return the faster/smaller direction.
        if (leftAngle < rightAngle)
        {
            return leftAngle;
        }
        else
        {
            return rightAngle * -1;
        }
    }

    private double checkOrientation()
    {
        Orientation ang = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return ang.firstAngle;
    }
}
