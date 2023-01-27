package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;

@TeleOp(name="Vuforia Testing", group="Testing")
public class TeleOpVuforiaTesting extends OpMode

{
    private CombinedLocalizer localizer        = null;
    private MecanumDriveBase mecanumDriveBase = null;
//    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private CameraInitSingleton cameraInitSingleton;
    private WebcamName webcam;

    public void init() {
        cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        webcam = cameraInitSingleton.getWebcam();
        telemetry.addData("Status", "Initialized");
        localizer = new CombinedLocalizer(hardwareMap, webcam);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        localizer = new CombinedLocalizer(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
//        towerController = new TowerController(hardwareMap);
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
        telemetry.addData("TeleOp heading", localizer.getHeading());
        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();
    }
}
