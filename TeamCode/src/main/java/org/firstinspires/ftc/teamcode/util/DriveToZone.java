package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "drive to zone", group = "Autonomous")
public class DriveToZone extends LinearOpMode
{
    EncoderMethod encoderMethod;
    private final double COUNTS_PER_MOTOR_GOBUILDA202 = 384.5 ;    // eg: TETRIX Motor Encoder REV    = 1440
    private final double DRIVE_GEAR_REDUCTION = 3.0 ;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    private final double inch = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor rf;

    @Override
    public void runOpMode() throws InterruptedException
    {
        {
            rb = hardwareMap.get(DcMotor.class, "rb");
            rf = hardwareMap.get(DcMotor.class, "rf");
            lb = hardwareMap.get(DcMotor.class, "lb");
            lf = hardwareMap.get(DcMotor.class, "lf");

            lf.setDirection(DcMotor.Direction.FORWARD);
            rf.setDirection(DcMotor.Direction.REVERSE);
            lb.setDirection(DcMotor.Direction.FORWARD);
            rb.setDirection(DcMotor.Direction.REVERSE);
            // Brake when power set to Zero
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        ConeDetector coneDetector = new ConeDetector();
        waitForStart();
        if (opModeIsActive())
        {
            int num = coneDetector.detector(telemetry, hardwareMap);

            telemetry.addData("Cone Detected is ", num);
            telemetry.update();
            drive(inch * 8, inch * 8, inch * 8, inch * 8, 0.5);
            Thread.sleep(1000);
            drive(-inch * 3, -inch * 3, -inch * 3, -inch * 3, 0.5);
            Thread.sleep(1000);
            if (1 == num)
            {
                drive(-inch * 12, inch * 12, inch * 12, -inch * 12, 0.5);
                telemetry.addData("", "%s", "if (1 == num)");
                telemetry.update();
            }
            if (2 == num)
            {
                drive(inch * 12, -inch * 12, -inch * 12, inch * 12, 0.5);
                telemetry.addData("", "%s", "if (1 == num)");
                telemetry.update();
            }
            if (3 == num)
            {
                drive(-inch * 30, inch * 30, inch * 30, -inch * 30, 0.5);
                telemetry.addData("", "%s", "if (1 == num)");
                telemetry.update();
            }
            Thread.sleep(1000);
            drive(inch * 20, inch * 20, inch * 20, inch * 20, 0.5);
            telemetry.addData("", "%s", "end");
            telemetry.update();

// 20 equals 18 inches

        }
    }

    public void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed)
    {
        int leftFrontPos = 0;
        int rightFrontPos = 0;
        int leftBackPos = 0;
        int rightBackPos = 0;

        leftFrontPos += leftFrontTarget;
        rightFrontPos += rightFrontTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;

        lf.setTargetPosition(leftFrontPos);
        rf.setTargetPosition(rightFrontPos);
        lb.setTargetPosition(leftBackPos);
        rb.setTargetPosition(rightBackPos);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        while (opModeIsActive() && lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())
        {
            idle();
        }
    }
}
