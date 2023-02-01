// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode.util;

import static com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;

@Autonomous(name="SamTest", group="Exercises")

public class SamIMUTest extends LinearOpMode
{
    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    private MecanumDriveBase mecanumDriveBase;
    private TeamDetection teamDetection;
    private ConeDetection coneDetection;
    private StateServer stateServer;

    private DcMotor screw;
    private DcMotor uBar;
    private DcMotor intake;

    private DistanceSensor distanceSensor;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;

    private int screwLevel;
    private int zone;
    private boolean blueSide;

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, strafeCorrection,  rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive, pidStrafe;

    BNO055IMU.Parameters myIMUparameters;

    private void initialize()
    {
//        teamDetection = new TeamDetection(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
        intake = hardwareMap.get(DcMotor.class, "intake");

        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");

        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID pro
        // portional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        pidStrafe = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        // wait for start button
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        //TODO: Do we do this just once?  Can the same settings be used for strafe?
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(-power, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

/*        //TODO: Same settings as drive????
        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(-power, power);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable(); */

        //Drive forward 18 inches
        while (opModeIsActive())
        {
            driveStraight(18 * ticksPerInch, 1, 0.1);
            rotate(180, 0.3);
            sleep(1500);
        }
    }

    //Set target then multiply by one with negative if you want to go backwards no negative input
    private void driveStraight(double target, int forward, double speed)
    {
        speed *= forward;

        //Fetch the odometry pod wheel location.
        leftFrontPos = mecanumDriveBase.lf.getCurrentPosition();
        if (forward == 1)
        {
            //Add the target distance to the current location
            leftFrontPos += target;

            //Drive from current position to target position
            while (mecanumDriveBase.lf.getCurrentPosition() <= leftFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                correction = pidDrive.performPID(getAngle());
//                correction *= 0.100;
                //Pass the correction value into the turn param.  No idea what kind of range will
                //be on this value.  Should be small value like 0.1 or less I would hope.
                mecanumDriveBase.driveMotors(speed, correction, 0, 1);

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.addData("5 lf pos", mecanumDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            leftFrontPos -= target;
            while (mecanumDriveBase.lf.getCurrentPosition() >= leftFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                correction = pidDrive.performPID(getAngle());

                mecanumDriveBase.driveMotors(speed, correction, 0, 1);

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.addData("5 lf pos", mecanumDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */    private double rotate(int degrees, double power)
    {
        // restart imu angle tracking.
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

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                mecanumDriveBase.driveMotors(0, power, 0, 1);

                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                mecanumDriveBase.driveMotors(0, power, 0, 1);

                double distance = checkDistance(5.0, 10.0);
                if (distance != -1)
                {
                    //WE SEE SOMETHING IN THE GIVEN RANGE.  STOP NOW!!!!!!
                    //    pidRotate.setSetpoint(pidRotate.getSetpoint());
                    //need to make this case enter only once
                }

            }
            while (opModeIsActive() && !pidRotate.onTarget());
        }
        else
        {
            // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                //leftMotor.setPower(-power);
                //rightMotor.setPower(power);
            }
            while (opModeIsActive() && !pidRotate.onTarget());
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();

        //TODO: return the number of degrees it did turn.  (in case distance preempted turn)
        return 0.0;
    }

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
    private void strafe(double target, int right, double speed)
    {
        speed *= right;
        if (right == 1)
        {
            rightFrontPos -= target;
            while (mecanumDriveBase.rf.getCurrentPosition() >= rightFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                strafeCorrection = pidStrafe.performPID(getAngle());

                mecanumDriveBase.driveMotors(0, strafeCorrection, speed, 1);
                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            rightFrontPos += target;
            while (mecanumDriveBase.rf.getCurrentPosition() <= rightFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                strafeCorrection = pidStrafe.performPID(getAngle());

                mecanumDriveBase.driveMotors(0, strafeCorrection, speed, 1);
                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
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

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //* SENSOR CODE   */
    ////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Check to see if the sensor detects an object within the range provided.
     * @param innerBound minimum range detection.
     * @param outerBound maximum range of detection.
     */
    private double checkDistance(double innerBound, double outerBound)
    {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        if (distance < outerBound && distance > innerBound)
        {
            //Will return the detected distance
        }
        else
        {
            distance = -1;
        }

        return distance;
    }

}
