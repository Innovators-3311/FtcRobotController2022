package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderMethod
{

    private final double     COUNTS_PER_MOTOR_GOBUILDA202    = 384.5 ;    // eg: TETRIX Motor Encoder REV    = 1440
    private final double     DRIVE_GEAR_REDUCTION    = 3.0 ;     // This is < 1.0 if geared UP
    private final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private double speed;

    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor rf;

    public void motorStart(HardwareMap hardwareMap)
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

    }

    // forward drive(1000, 1000, 1000, 1000, 0.25);
    // Turn drive(1000, -1000, 1000, -1000, 0.25);
    // strafe drive(1000, -1000, -1000, 1000, 0.25);
    //for GoBUILDA drive bases
}