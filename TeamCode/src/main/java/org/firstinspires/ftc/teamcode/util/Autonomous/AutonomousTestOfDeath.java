package org.firstinspires.ftc.teamcode.util.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.ConeDetection;
import org.firstinspires.ftc.teamcode.util.DriveToPos;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TeamDetection;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousTestOfDooooooooooooooooooooom", group = "autonomous")
@Disabled
public class AutonomousTestOfDeath extends LinearOpMode
{
    public CombinedLocalizer combinedLocalizer;
    private DriveToPos driveToPos;
    private MecanumDriveBase mecanumDriveBase;
    private CameraInitSingleton cameraInitSingleton;
    private TeamDetection teamDetection;
    private ConeDetection coneDetection;
    private StateServer stateServer;

    private DcMotor screw;
    private DcMotor uBar;
    private DcMotor intake;

    private DistanceSensor distanceSensor;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;
    private WebcamName webcam;

    private int screwLevel;
    private int zone = -1;
    private boolean blueSide;

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;

    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

//    private final double ticksPerDegree = ticksPerInch * 4.75 / 90;
    private final double ticksPerDegree = ticksPerInch * 4.77 / 90;

    @Override
    public void runOpMode() throws InterruptedException
    {
//        try {
//            stateServer = new StateServer();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
        cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        webcam = cameraInitSingleton.getWebcam();
        combinedLocalizer = new CombinedLocalizer(hardwareMap, webcam);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        driveToPos = new DriveToPos(combinedLocalizer, mecanumDriveBase);

        telemetry.addData("Intialized", "");
        telemetry.update();

        while (!opModeIsActive())
        {
//            zone = coneDetection.detector(telemetry, hardwareMap);
            waitForStart();
        }
        if (zone == -1)
        {
            zone = 2;
        }
        //crunch crunch crunch
        // Waits till start button is pressed
//        coneDetection.tfod.setZoom(1, 16/9);
        combinedLocalizer.handleTracking();
        RobotLog.ii("AutomousTestofDeath", "State= %.2f %.2f %.2f", combinedLocalizer.x, combinedLocalizer.y, combinedLocalizer.heading);
        ElapsedTime runtime = new ElapsedTime();
        runtime.seconds();
        runtime.startTime();

//        encoderLogging();
//        zone = coneDetection.detector(telemetry, hardwareMap);
//        blueSide = teamDetection.showTeam(telemetry);

//        while (zone == 0)
//        {
//            idle();
//        }

        /******  Take 45 */
//        driveScrew(200);
//        driveUBar(1750);

//        zone = coneDetection.detector(telemetry, hardwareMap, webcam);
//        blueSide = teamDetection.showTeam(telemetry);
//        blueSide = false;
        telemetry.addData("", "On blue side? " + blueSide + " parking zone is equal to " + zone);
        telemetry.update();

        /************************/
        //Drive to first pole
        driveStraight(ticksPerInch * 24, 1, 0.5);
        turnInPlace(ticksPerDegree * 800, 1, 0.5); // FIXME change the negative
        Thread.sleep(1000);
//        turnInPlace(ticksPerDegree * 90 / 4, -1, 0.5); // FIXME change the negative
        driveToPos.setTarget(112, 36, -90);
        driveToPos.maxSpeedFactor = 0.4;
        double startTime = runtime.seconds();
        while ((driveToPos.distanceToTarget() >= 5) && combinedLocalizer.targetVisible) // && (runtime.seconds() + startTime < 100))
        {
            combinedLocalizer.handleTracking();
            driveToPos.autoDriveToPosition(telemetry);
            RobotLog.ii("AutonomousTestOfDeath", "drivetopos distance: %f heading: %f, pos unc: %f, vuforiaTargetVisible: %s",
                    driveToPos.distanceToTarget(), combinedLocalizer.heading, combinedLocalizer.positionUncertainty,
                    String.valueOf(combinedLocalizer.targetVisible));
            telemetry.update();
        }

        // Stop!
        mecanumDriveBase.driveMotors(0,0,0,0);

        stop();

        Thread.sleep(1000000000);

//        driveScrew(200);
        driveStraight(ticksPerInch * 30, 1, 0.3);
        Thread.sleep(1000);
        driveStraight(ticksPerInch * 6, -1, 0.3);
//        driveUBar(1750);
        Thread.sleep(1000);
        //Turn to middle poll
        if (blueSide)
        {
            turnInPlace(ticksPerDegree * 45, -1, 0.3);
        }
        else
        {
            turnInPlace(ticksPerDegree * 43, 1, 0.3);
        }
        Thread.sleep(1000);
        driveStraight(ticksPerInch * 4, 1, 0.5);
        Thread.sleep(1000);
        intake.setPower(-1);
        Thread.sleep(1000);
        intake.setPower(0);

        // pickup cone
//        driveScrew(100);
        driveStraight(ticksPerInch * 4, -1, 0.5);
        Thread.sleep(500);
//        driveUBar(0);
        if (blueSide)
        {
            turnInPlace(ticksPerDegree * 45, -1, 0.3);
        }
        else
        {
            turnInPlace(ticksPerDegree * 55, 1, 0.3);
        }
        Thread.sleep(500);
//        driveStraight(ticksPerInch * 30, 1, 0.5);
//        Thread.sleep(500);
//        driveStraight(ticksPerInch * 5, -1, 0.5);


//        Thread.sleep(500);
//        if (blueSide)
//        {
//            turnInPlace(ticksPerDegree * 90, 1, 0.3);
//        }
//        else
//        {
//            turnInPlace(ticksPerDegree * 90, -1, 0.3);
//        }
//        Thread.sleep(500);
//        driveStraight(ticksPerInch * 2, 1, 0.5);
//        Thread.sleep(500);

        switch (zone)
        {
            case 1:
                if (blueSide)
                {
                    driveStraight(ticksPerInch * 22, 1, 0.5);
                }
                else
                {
                    driveStraight(ticksPerInch * 22, -1, 0.5);
                }
                break;

            case 2:
                if (blueSide)
                {

                }
                else
                {

                }
                break;

            case 3:
                if (blueSide)
                {
                    driveStraight(ticksPerInch * 22, -1, 0.5);
                }
                else
                {
                    driveStraight(ticksPerInch * 22, 1, 0.5);
                }
                break;
        }


//        Thread.sleep(500);
//        driveStraight(ticksPerInch * 18, 1, 0.5);



//        //drive to first pole
//        driveScrew(3300);
//        driveStraight(ticksPerInch * 56, 1, 0.6);
//        driveUBar(1750);
//        Thread.sleep(750);
//
//        driveStraight(ticksPerInch * 5, -1, 0.6);
//        Thread.sleep(750);
//
//        if (blueSide)
//        {
//            turnInPlace(ticksPerDegree * 45, -1, 0.6);
//        }
//        else
//        {
//            turnInPlace(ticksPerDegree * 38, 1, 0.6);
//        }
//        Thread.sleep(750);
//
//        driveStraight(ticksPerInch * 5, 1, 0.6);
//        mecanumDriveBase.driveMotors(0, 0, 0, 0);
//        Thread.sleep(1000);
//
//        //Drop cone
//        intake.setPower(-1);
//        Thread.sleep(1000);
//
//        driveUBar(2000);
//        intake.setPower(0);
//
//        // pickup another cone
//        driveStraight(ticksPerInch * 2.5, -1, 0.6);
//        Thread.sleep(750);
//
//        if (blueSide)
//        {
//            turnInPlace(ticksPerDegree * 135.2, 1, 0.6);
//        }
//        else
//        {
//            turnInPlace(ticksPerDegree * 132.2, -1, 0.6);
//        }
//
//        driveUBar(500);
//        Thread.sleep(750);
//
//        driveScrew(1900);
//        driveStraight(ticksPerInch * 20.3, 1, 0.6);
//        driveScrew(980);
//        Thread.sleep(500);
//
//        intake.setPower(1);
//        Thread.sleep(1250);
//
//        intake.setPower(0);
//        driveScrew(2500);
//        Thread.sleep(1000);
//
//        //Deliver second cone
//        uBar.setTargetPosition(-3000);
//        driveStraight(ticksPerInch * 17, -1, 0.6);
//        Thread.sleep(500);
//
//        screw.setTargetPosition(2800);
//        if (blueSide)
//        {
//            turnInPlace(ticksPerDegree * 45, 1, 0.6);
//        }
//        else
//        {
//            turnInPlace(ticksPerDegree * 45, -1, 0.6);
//        }
//        Thread.sleep(500);
//
//        driveStraight(ticksPerInch * 6.7, -1, 0.6);
//        Thread.sleep(500);
//
//        intake.setPower(-1);
//        Thread.sleep(1500);
//
//        intake.setPower(0);
//        Thread.sleep(1000);
//
//        //Park
//        driveStraight(ticksPerInch * 6, 1, 0.6);
//        Thread.sleep(500);
//
//        driveUBar(0);
//        if (blueSide)
//        {
//            turnInPlace(ticksPerDegree * 37, -1, 0.6);
//        }
//        else
//        {
//            turnInPlace(ticksPerDegree * 33.5, 1, 0.6);
//        }
//        Thread.sleep(500);
//
//        if (blueSide)
//        {
//            switch (zone)
//            {
//                case 1:
//                    driveUBar(0);
//                    driveStraight(ticksPerInch * 20, -1, 0.6);
//                    driveScrew(0);
//                    break;
//
//                case 2:
//                    driveUBar(0);
//                    driveScrew(0);
//                    break;
//
//                case 3:
//                    driveUBar(0);
//                    driveStraight(ticksPerInch * 20, 1, 0.6);
//                    driveScrew(0);
//                    break;
//
//                default:
//                    driveUBar(0);
//                    driveStraight(ticksPerInch * 19, 1, 0.61);
//                    driveScrew(0);
//                    break;
//            }
//        }
//        else
//        {
//            switch (zone)
//            {
//                case 1:
//                    driveUBar(0);
//                    driveStraight(ticksPerInch * 20, 1, 0.6);
//                    driveScrew(0);
//                    break;
//
//                case 2:
//                    driveUBar(0);
//                    driveScrew(0);
//                    break;
//
//                case 3:
//                    driveUBar(0);
//                    driveStraight(ticksPerInch * 20, -1, 0.6);
//                    driveScrew(0);
//                    break;
//
//                default:
//                    driveUBar(0);
//                    driveStraight(ticksPerInch * 20, -1, 0.6);
//                    driveScrew(0);
//                    break;
//            }
//        }

        stop();

/*
        if (blueSide)
        {
            driveStrafe(ticksPerInch * 24, -1, 0.5);
        }
        else
        {
            driveStrafe(ticksPerInch * 24, 1, 0.5);
        }
        Thread.sleep(500);
        driveUBar(-3250);
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
        telemetry.addData("", runtime.seconds());
        telemetry.update();
        while (runtime.seconds() < 18)
        {
            mecanumDriveBase.driveMotors(0.1,0, 0, -1);
        }
        driveUBar(-3050);
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
        Thread.sleep(1000);
        intake.setPower(-1);
        Thread.sleep(1000);
        intake.setPower(0);

        driveStraight(ticksPerInch * 2.5, 1, 0.5);
        Thread.sleep(500);
        driveUBarSpecial(-600);
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
        driveUBar(0);
        driveScrew(0);
        driveStraight(ticksPerInch * 4, -1, 0.3);
        while (screw.isBusy())
        {
            idle();
        }
        // Stops program when reached
        stop();

 */
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
                combinedLocalizer.handleTracking();
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
                combinedLocalizer.handleTracking();
                mecanumDriveBase.driveMotors(speed, 0, 0, 1);
                telemetry.addData("", mecanumDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
        combinedLocalizer.handleTracking();
//        encoderLogging();
    }

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
//    private void driveStrafe2(double target, int right, double strafeSpeed)
//    {
//        double correctionStart = mecanumDriveBase.lb.getCurrentPosition();
//
//        //negative is left??
//        strafeSpeed *= right;
//
//        if (right == 1)
//        {
//            rightFrontPos -= target;
//            while (mecanumDriveBase.rf.getCurrentPosition() >= rightFrontPos)
//            {
//                double drift = correctionStart - mecanumDriveBase.lb.getCurrentPosition();
//
//                mecanumDriveBase.driveMotors(drift, 0, strafeSpeed, 0.5);
//                //mecanumDriveBase.driveMotors(0, 0, speed, 1);
//                telemetry.addData("Drift = ", drift);
//                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//        else
//        {
//            rightFrontPos += target;
//            while (mecanumDriveBase.rf.getCurrentPosition() <= rightFrontPos)
//            {
//                double drift = correctionStart - mecanumDriveBase.lb.getCurrentPosition();
//
//                mecanumDriveBase.driveMotors(drift, 0, strafeSpeed, 0.5);
//                //mecanumDriveBase.driveMotors(0, 0, speed, 1);
//                telemetry.addData("Drift = ", drift);
//                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//        mecanumDriveBase.driveMotors(0, 0, 0, 0);
////        encoderLogging();
//    }
//
//    //Set target then multiply by one with negative if you want to go left currently set right no negative input
//    private void driveStrafe(double target, int right, double speed)
//    {
//        speed *= right;
//        if (right == 1)
//        {
//            rightFrontPos -= target;
//            while (mecanumDriveBase.rf.getCurrentPosition() >= rightFrontPos)
//            {
//                mecanumDriveBase.driveMotors(0, 0, speed, 1);
//                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//        else
//        {
//            rightFrontPos += target;
//            while (mecanumDriveBase.rf.getCurrentPosition() <= rightFrontPos)
//            {
//                mecanumDriveBase.driveMotors(0, 0, speed, 1);
//                telemetry.addData("", mecanumDriveBase.rf.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//        mecanumDriveBase.driveMotors(0, 0, 0, 0);
////        encoderLogging();
//    }

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
    private void turnInPlace(double target, int right, double speed)
    {
        speed *= right;

        leftBackPos = mecanumDriveBase.lb.getCurrentPosition();
        telemetry.addData("target = ", target);
        telemetry.addData("lb pos = ", mecanumDriveBase.lb.getCurrentPosition());
        telemetry.addData("rf pos = ", mecanumDriveBase.rf.getCurrentPosition());
        telemetry.update();

        if (right == 1)
        {
            leftBackPos -= target; // fixme CHANG SPEED
            while (mecanumDriveBase.lb.getCurrentPosition() >= leftBackPos)
            {
                mecanumDriveBase.driveMotors(0, speed, 0, -1);
                combinedLocalizer.handleTracking();
                telemetry.addData("target = ", target);
                telemetry.addData("lb pos = ", mecanumDriveBase.lb.getCurrentPosition());
                telemetry.addData("rf pos = ", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();

            }
        }
        else
        {
            leftBackPos += target; // fixme CHANG SPEED
            while (mecanumDriveBase.lb.getCurrentPosition() <= leftBackPos)
            {
                mecanumDriveBase.driveMotors(0, speed, 0, -1);
                combinedLocalizer.handleTracking();
                telemetry.addData("target = ", target);
                telemetry.addData("lb pos = ", mecanumDriveBase.lb.getCurrentPosition());
                telemetry.addData("rf pos ", mecanumDriveBase.rf.getCurrentPosition());
                telemetry.update();

            }
        }
        combinedLocalizer.handleTracking();
        mecanumDriveBase.driveMotors(0, 0, 0, 0);
    }

//    private void driveScrew(int target)
//    {
//        screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        screw.setTargetPosition(target);
//        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        screw.setPower(1);
//    }
//
//    private void driveUBar(int target)
//    {
//        uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        uBar.setTargetPosition(target);
//        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        uBar.setPower(1);
//    }

//    private void driveUBarSpecial(int target) throws InterruptedException
//    {
//        Thread.sleep(500);
//        uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        uBar.setTargetPosition(target);
//        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        uBar.setPower(1);
//    }

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


