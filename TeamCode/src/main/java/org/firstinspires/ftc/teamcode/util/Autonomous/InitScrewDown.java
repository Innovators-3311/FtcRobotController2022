package org.firstinspires.ftc.teamcode.util.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.ConeDetection;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.TeamDetection;

@Autonomous(name="screwInitTeleOp ", group="Exercises")
public class InitScrewDown extends LinearOpMode
{
    private CameraInitSingleton cameraInitSingleton;
    // private WebcamName webcam;
    private ConeDetection coneDetection;
    private MecanumDriveBase mecanumDriveBase;
    private ElapsedTime elapsedTime;
    private TeamDetection teamDetection;

    private TouchSensor lowSensor;
    private TouchSensor highSensor;

    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorCenter;

    private DcMotor screw;
    private DcMotor uBar;
    private DcMotor intake;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double globalAngle;
    double correction;
    double rotation;
    double initAngle;

    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    double distance;
    double toPole = 0;
    double leftFrontPos;
    double RightFrontPos;
    double LeftBackPos;

    boolean blueTeam = true;
    int zone = -1; // TODO init these values

    private int screwLevel;


    PIDController pidRotate, pidDrive, pidStrafe;

    // called when init button is  pressed.

    private void initialize()
    {
        cameraInitSingleton = new CameraInitSingleton(hardwareMap);

        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        coneDetection = new ConeDetection(hardwareMap, cameraInitSingleton.getWebcam());
        teamDetection = new TeamDetection(hardwareMap);
        elapsedTime = new ElapsedTime();

        distanceSensorCenter = hardwareMap.get(DistanceSensor.class, "distanceSensorCenter");

        //Map Hardware
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");
        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");

        screw.setDirection(DcMotor.Direction.REVERSE);
        uBar.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hit start", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Initialize
        initialize();

//        testScrew();

        //Detecting the code before waitForStart may have been causing us issues.  At any point it
        //is not needed here, and the init preview still works and will ID the code that it sees.
//        zone = coneDetection.detector(telemetry);
        RobotLog.ii("Auto", "Waiting for start..");

        telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        driveScrew(0);

//        while(opModeIsActive())
//        {
//            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
//            telemetry.update();
//        }
//
//        //        while (opModeIsActive())
//        {
//            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
//            telemetry.update();
//            driveScrew(800);
//            sleep(5000);
//            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
//            telemetry.update();
//            driveScrew(500);
//            sleep(5000);
//            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
//            telemetry.update();
//        }
//
//
//        while(opModeIsActive())
//        {
//            telemetry.addData("Screw Pos: Lowering", screw.getCurrentPosition());
//            telemetry.update();
//
//
//            // Sets direction
//            screw.setDirection(DcMotor.Direction.REVERSE);
//            screw.setTargetPosition(screwLevel);
//            // sets run mode
//            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // sets power
//            screw.setPower(0.05);
//
//            while ( !(lowSensor.isPressed()))
//            {
//                telemetry.addData("Screw Pos: Lowering", screw.getCurrentPosition());
//                telemetry.update();
//
//                // Stops if sensor is true
//                if (lowSensor.isPressed())
//                {
//
//                    break;
//                }
//            }
//
//            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


/*
            driveScrewDown(10000,0.2);
            int lowPos = screw.getCurrentPosition();
            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
            telemetry.update();
            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(4000);

            telemetry.addData("Screw Pos: rising", screw.getCurrentPosition());
            telemetry.update();
            driveScrewUp(50000, 0.2);
            int highPos = screw.getCurrentPosition();
            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
            telemetry.update();

            sleep(4000);

//            screw.setTargetPosition(highPos/2);
//            screw.setPower(0.2);
//            telemetry.addData("Screw Pos: ", screw.getCurrentPosition());
//            telemetry.update();
*/
        stop();
    }

    private void driveScrew(int target)
    {
        screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        screw.setTargetPosition(target);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        screw.setPower(1);
    }
}
