package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "autonomous")
public class Autonomous extends LinearOpMode
{
    private MecanumDriveBase mecanumDriveBase;
    private TeamDetection teamDetection;
    private ConeDetection coneDetection;
    private StateServer stateServer;
    private TowerController towerController;

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

    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    @Override
    public void runOpMode() throws InterruptedException
    {
//        try {
//            stateServer = new StateServer();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
        teamDetection = new TeamDetection(hardwareMap);
        coneDetection = new ConeDetection();
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
//        towerController = new TowerController(hardwareMap, telemetry);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
        intake = hardwareMap.get(DcMotor.class, "intake");

        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");

        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

            // Run Without Encoders
//        mecanumDriveBase.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mecanumDriveBase.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mecanumDriveBase.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mecanumDriveBase.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Brake when power set to Zero
//        mecanumDriveBase.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mecanumDriveBase.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mecanumDriveBase.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mecanumDriveBase.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveScrewUp(500, 0.5);
        driveScrewDown(10000, 0.5);

        screw.setDirection(DcMotorSimple.Direction.REVERSE);

        zone = coneDetection.detector(telemetry, hardwareMap);
        blueSide = teamDetection.showTeam(telemetry);

//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distanceSensor;

        telemetry.addData("Hit", "start when ready", "");
        telemetry.addData("", "On blue side? " + blueSide + " parking zone is equal to " + zone);
        telemetry.update();

        // Waits till start button is pressed
        waitForStart();

//        encoderLogging();

        //drive to first pole

        driveScrew(2720);
        driveStrafe(ticksPerInch * 24, -1, .5);
        Thread.sleep(500);
        driveUBar(-3109);
        driveStraight(ticksPerInch * 49, 1, 0.5); // 63,847
        Thread.sleep(500);
        //strafes to pole
        //        driveStrafe(ticksPerInch * 10.2, -1, .5);
        if (blueSide)
        {
            driveStrafe(ticksPerInch * 12, -1, 0.5);
//            while (distanceSensor.getDistance(DistanceUnit.INCH) < 3.75)
//            {
//                mecanumDriveBase.driveMotors(0, 0, -0.5, 1);
//            }
        }
        else
        {
            driveStrafe(ticksPerInch * 12, 1, 0.5);
//            while (distanceSensor.getDistance(DistanceUnit.INCH) < 3.75)
//            {
//                mecanumDriveBase.driveMotors(0, 0, 0.5, 1);
//            }
        }
        Thread.sleep(500);

//        driveStraight((ticksPerInch * distanceSensor.getDistance(DistanceUnit.INCH)) - 0.75, -1, 0.5);
        driveStraight(ticksPerInch * 3, -1, 0.5);
        Thread.sleep(500);
        intake.setPower(-1);
        Thread.sleep(1000);
        intake.setPower(0);
        driveStraight(ticksPerInch * 3, 1, 0.5);

        switch (zone)
        {
            case 1:
                if (blueSide)
                {
                    driveStrafe(ticksPerInch * 8, 1, 0.5);
                }
                else
                {
                    driveStrafe(ticksPerInch * 56, -1, 0.5);
                }
            break;

            case 2:
                if (blueSide)
                {
                    driveStrafe(ticksPerInch * 32, 1, 0.5);
                }
                else
                {
                    driveStrafe(ticksPerInch * 32, -1, 0.5);
                }
                break;

            case 3:
                if (blueSide)
                {
                    driveStrafe(ticksPerInch * 56, 1, 0.5);
                }
                else
                {
                    driveStrafe(ticksPerInch * 8, -1, 0.5);
                }
                break;
        }
        // Stops program when reached
        stop();
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

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
    private void driveStrafe(double target, int right, double speed)
    {
        speed *= right;

        if (right == 1)
        {
            rightFrontPos -= target;
            while (mecanumDriveBase.rf.getCurrentPosition() >= rightFrontPos)
            {
                mecanumDriveBase.driveMotors(0, 0, speed, 1);
                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            rightFrontPos += target;
            while (mecanumDriveBase.rf.getCurrentPosition() <= rightFrontPos)
            {
                mecanumDriveBase.driveMotors(0, 0, speed, 1);
                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
    private void turnInPlace(double target, int right, double speed)
    {
        target *= right;

//        mecanumDriveBase.lf.setPower(right * speed);
//        mecanumDriveBase.rf.setPower(-right * speed);
//        mecanumDriveBase.lb.setPower(-right * speed);
//        mecanumDriveBase.rb.setPower(right * speed);

        while (mecanumDriveBase.lb.getCurrentPosition() != target){}
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

//    private void encoderLogging()
//    {
//        try {
//            JSONObject state = new JSONObject()
//                    .put("lf encoder", mecanumDriveBase.lf.getCurrentPosition())
//                    .put("rf encoder", mecanumDriveBase.rf.getCurrentPosition())
//                    .put("lb encoder", mecanumDriveBase.lb.getCurrentPosition());
//            stateServer.addState(state);
//        } catch (JSONException e) {
//            RobotLog.ee("Localizer", "Error encoding json.");
//        };
//    }

    private void driveScrewUp(double screwTarget, double speed)
    {
        screwLevel -= screwTarget;
        // Sets direction
        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setTargetPosition(screwLevel);
        // sets run mode
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // sets power
        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            // Stops if sensor is true
            if (highSensor.isPressed())
            {
                break;
            }
        }

        // breaks the motor
        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }

    private void driveScrewDown(double screwTarget, double speed)
    {
        // Sets target position
        screwLevel -= screwTarget;

        // Sets direction
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        // sets run mode
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // sets power
        screw.setPower(-speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (lowSensor.isPressed())))
        {
            // Stops if sensor is true
            if (lowSensor.isPressed())
            {
                break;
            }
        }

        // breaks the motor
        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }
}
