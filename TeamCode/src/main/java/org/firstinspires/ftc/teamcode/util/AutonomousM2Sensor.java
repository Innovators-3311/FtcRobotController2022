package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonomousM2Sensor extends LinearOpMode
{
    private MecanumDriveBase mecanumDriveBase;
    private DistanceSensor distanceSensor;
    private ElapsedTime elapsedTime;

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;

    private double offset = 3;

    static final double     COUNTS_PER_MOTOR_GOBUILDA202    = 384.5 ;    // eg: TETRIX Motor Encoder REV    = 1440
    static final double     DRIVE_GEAR_REDUCTION    = 3.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);


    public void initialization()
    {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialization();

        waitForStart();

        drive(COUNTS_PER_INCH * 12, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, 0.2);

        while (distanceSensor.getDistance(DistanceUnit.INCH) + offset > 0.2)
        {
            if (distanceSensor.getDistance(DistanceUnit.INCH) + offset > 12)
            {
                findPole();
            }
            telemetry.addData("Distance to pole", distanceSensor.getDistance(DistanceUnit.INCH) + offset);
            mecanumDriveBase.driveMotors(1, 0, 0, 0.1);

        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
        stop();
    }

    private void findPole()
    {
        elapsedTime.reset();
        elapsedTime.startTime();
        while (elapsedTime.seconds() < 3 || distanceSensor.getDistance(DistanceUnit.INCH) + offset < 12)
        {
            mecanumDriveBase.driveMotors(0, 1, 0, 0.1);
        }
        while (distanceSensor.getDistance(DistanceUnit.INCH) + offset < 12)
        {
            mecanumDriveBase.driveMotors(0, -1, 0, 0.1);
        }
    }

    private void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed)
    {
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

        while (opModeIsActive() && mecanumDriveBase.lf.isBusy() && mecanumDriveBase.rf.isBusy() && mecanumDriveBase.lb.isBusy() && mecanumDriveBase.rb.isBusy())
        {
            idle();
        }


    }
}