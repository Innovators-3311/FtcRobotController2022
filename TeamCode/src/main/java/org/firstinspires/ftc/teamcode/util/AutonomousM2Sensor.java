package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RevM2 Auto Test", group = "doom")
public class AutonomousM2Sensor extends LinearOpMode {
    private MecanumDriveBase mecanumDriveBase;
    private ElapsedTime elapsedTime;

    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;

    private double offset = 3;


    static final double COUNTS_PER_MOTOR_GOBUILDA202 = 384.5;    // eg: TETRIX Motor Encoder REV    = 1440
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public void initialization()
    {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        elapsedTime = new ElapsedTime();
        telemetry.addData("Initialized", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialization();

        waitForStart();

        drive(COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, 0.2);

//        elapsedTime.startTime();
//
//        drive(COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, 0.2);
//
//        mecanumDriveBase.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        mecanumDriveBase.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        mecanumDriveBase.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        mecanumDriveBase.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        while (distanceSensor.getDistance(DistanceUnit.INCH) + offset > 0.2)
//        {
//            if (distanceSensor.getDistance(DistanceUnit.INCH) + offset > 24)
//            {
//                findPole();
//            }
//            telemetry.addData("Distance to pole", distanceSensor.getDistance(DistanceUnit.INCH) + offset);
//            telemetry.update();
//            mecanumDriveBase.driveMotors(1, 0, 0, 0.1);
//        }
//        telemetry.addData("Found pole", "");
//        telemetry.update();
//        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        stop();
    }

//    private void findPole()
//    {
//        telemetry.addData("Swerving", "");
//        telemetry.update();
//        elapsedTime.reset();
//        elapsedTime.startTime();
//
//        //while time is less than 3 seconds OR pole is more than 15 inches away
//        while (elapsedTime.seconds() < 3 || distanceSensor.getDistance(DistanceUnit.INCH) + offset > 12)
//        {
//            mecanumDriveBase.driveMotors(0, 1, 0, 0.1);
//        }
//
//        //If pole is greater than 24 inches away
//        if (distanceSensor.getDistance(DistanceUnit.INCH) + offset > 24)
//        {
//            //Turn while pole is greater than 24 inches away
//            while (distanceSensor.getDistance(DistanceUnit.INCH) + offset > 24)
//            {
//                mecanumDriveBase.driveMotors(0, -1, 0, 0.1);
//            }
//        }
//    }

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