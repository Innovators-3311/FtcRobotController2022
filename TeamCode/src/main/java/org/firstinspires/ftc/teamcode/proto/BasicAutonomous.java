package org.firstinspires.ftc.teamcode.proto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;

@Autonomous(name = "basicAutonomous", group = "2022 - 2023 Autonomous")

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

        turnInPlace(5000, 1, 1);

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


        mecanumDriveBase.driveMotors(0, speed, 0, 1);
    while (mecanumDriveBase.lb.getCurrentPosition() !=  target){
        mecanumDriveBase.driveMotors(0, speed, 0, 1);
    }
    mecanumDriveBase.driveMotors(0, 0, 0, 0);
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
