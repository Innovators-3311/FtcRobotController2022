package org.firstinspires.ftc.teamcode.proto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.SimplePIDControl;

@Autonomous(name = "basicMezAutonomous", group = "2022 - 2023 Autonomous")

public class BasicAutonomous extends LinearOpMode
{
    private MecanumDriveBase mecanumDriveBase;
    // Initialize motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    static final double     COUNTS_PER_MOTOR_GOBUILDA202    = 384.5 ;
    static final double     DRIVE_GEAR_REDUCTION    = 3.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBUILDA202 * 1) /
                                                      (4 * 3.1415);

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;

    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    private final double degree = ticksPerInch * 4.8 / 90;

    //Four modes
    //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    @Override
    public void runOpMode() throws InterruptedException
    {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;

        waitForStart();

        driveStraight(ticksPerInch * 55, 1, 0.5);

        turnInPlace(degree * 45, 1, 0.5);

        driveStraight(ticksPerInch * 5, 1, 0.2);


//        driveStrafe2(ticksPerInch * 50, 1, 0.5);

//        turnInPlace(5000, 1, 1);

//        drive(COUNTS_PER_INCH * 8, COUNTS_PER_INCH * 8, COUNTS_PER_INCH * 8, COUNTS_PER_INCH * 8, 0.25);
//        Thread.sleep(1000);
//        drive(-COUNTS_PER_INCH * 8, -COUNTS_PER_INCH * 8, -COUNTS_PER_INCH * 8, -COUNTS_PER_INCH * 8, 0.50);
//        Thread.sleep(1000);
//        drive(-1000, 1000, 1000, -1000, 100);



        // forward drive(1000, 1000, 1000, 1000, 0.25);
        // Turn drive(1000, -1000, 1000, -1000, 0.25);
        // strafe drive(1000, -1000, -1000, 1000, 0.25);
        stop();
    }
    private void turnInPlace(double target, int right, double speed)
    {
    target *= right;

    target = target + mecanumDriveBase.lb.getCurrentPosition();


        telemetry.addData("target = ", target);
        telemetry.addData("rf pos = ", mecanumDriveBase.lb.getCurrentPosition());
        telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
        telemetry.update();

    while (mecanumDriveBase.lb.getCurrentPosition() <=  target)
    {
        mecanumDriveBase.driveMotors(0, -speed, 0, 1);

        telemetry.addData("target = ", target);
        telemetry.addData("rf pos = ", mecanumDriveBase.lb.getCurrentPosition());
        telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
        telemetry.update();

    }

    mecanumDriveBase.driveMotors(0, 0, 0, 0);
    }

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
    private void driveStrafe2(double target, int right, double strafeSpeed)
    {
        double correctionStart = mecanumDriveBase.lb.getCurrentPosition();

        SimplePIDControl pidControl = new SimplePIDControl(1.0/ticksPerInch,0,0);


        //negative is left??
        strafeSpeed *= right;

        if (right == 1)
        {
            rightFrontPos -= target;
            while (mecanumDriveBase.rf.getCurrentPosition() >= rightFrontPos)
            {
                double drift = correctionStart - mecanumDriveBase.lb.getCurrentPosition();

                double forward = pidControl.update(drift);
                double clampForward = pidControl.clamp(forward, -1, 1);
                mecanumDriveBase.driveMotors(1, 0.00, 0, 1);
//                mecanumDriveBase.driveMotors(clampForward * 0.5, 0.00, strafeSpeed, 1);
                //mecanumDriveBase.driveMotors(0, 0, speed, 1);


                telemetry.addData("rf pos = ", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.addData("Drift = ", drift);
                telemetry.addData("forward = ", forward);
                telemetry.addData("clampforward = ", clampForward);
                telemetry.addData("strafeSpeed = ", strafeSpeed);
                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            rightFrontPos += target;
            while (mecanumDriveBase.rf.getCurrentPosition() <= rightFrontPos)
            {
                double drift = correctionStart - mecanumDriveBase.lb.getCurrentPosition();

                mecanumDriveBase.driveMotors(drift, 0, strafeSpeed, 0.5);
                //mecanumDriveBase.driveMotors(0, 0, speed, 1);
                telemetry.addData("Drift = ", drift);
                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }

    //Set target then multiply by one with negative if you want to go backwards no negative input
    private void driveStraight(double target, int forward, double speed)
    {
        speed *= forward;
        leftFrontPos = mecanumDriveBase.lf.getCurrentPosition();
        if (forward == 1)
        {
            leftFrontPos += target;
            while (mecanumDriveBase.lf.getCurrentPosition() <= leftFrontPos)
            {
                mecanumDriveBase.driveMotors(speed, 0, 0, 1);
                telemetry.addData("", mecanumDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            leftFrontPos -= target;
            while (mecanumDriveBase.lf.getCurrentPosition() >= leftFrontPos)
            {
                mecanumDriveBase.driveMotors(speed, 0, 0, 1);
                telemetry.addData("", mecanumDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }


    private void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed)
    {
        leftFrontPos += leftFrontTarget;
        rightFrontPos += rightFrontTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;

        leftFront.setTargetPosition(leftFrontPos);
        rightFront.setTargetPosition(rightFrontPos);
        leftBack.setTargetPosition(leftBackPos);
        rightBack.setTargetPosition(rightBackPos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())
        {
            idle();
        }

    }
}
