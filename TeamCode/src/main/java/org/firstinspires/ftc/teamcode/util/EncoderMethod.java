package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderMethod
{

    private final double     COUNTS_PER_MOTOR_GOBUILDA202    = 384.5 ;    // eg: TETRIX Motor Encoder REV    = 1440
    private final double     DRIVE_GEAR_REDUCTION    = 3.0 ;     // This is < 1.0 if geared UP
    private final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private  double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private double speed;
    private MecanumDriveBase mecanumDriveBase;

    public double getCOUNTS_PER_INCH()
    {
        return COUNTS_PER_INCH;
    }

    public double getSpeed()
    {
        return speed;
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

        mecanumDriveBase.lf.setTargetPosition(leftFrontPos);
        mecanumDriveBase.rf.setTargetPosition(rightFrontPos);
        mecanumDriveBase.lb.setTargetPosition(leftBackPos);
        mecanumDriveBase.rb.setTargetPosition(rightBackPos);

        mecanumDriveBase.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDriveBase.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDriveBase.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDriveBase.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       mecanumDriveBase.lf.setPower(speed);
       mecanumDriveBase.rf.setPower(speed);
       mecanumDriveBase.lb.setPower(speed);
       mecanumDriveBase.rb.setPower(speed);

    }
}