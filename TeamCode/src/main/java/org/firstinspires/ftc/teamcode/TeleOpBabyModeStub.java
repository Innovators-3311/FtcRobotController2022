package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;
import org.firstinspires.ftc.teamcode.util.controllers.BabyModeController;
import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;

@TeleOp(name="Baby Stub", group="Basic drive")
public class TeleOpBabyModeStub extends OpMode

{
    private CombinedLocalizer localizer        = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private BabyModeController babyModeController = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private CameraInitSingleton cameraInitSingleton;
    private WebcamName webcam;

    public void init() {
        cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        webcam = cameraInitSingleton.getWebcam();
        telemetry.addData("Status", "Initialized");
        localizer          = new CombinedLocalizer(hardwareMap, webcam);
        mecanumDriveBase   = new MecanumDriveBase(hardwareMap, false, webcam);
        babyModeController = new BabyModeController(mecanumDriveBase, localizer);
        towerController    = new TowerController(hardwareMap,telemetry);
        pacMan             = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
        telemetry.addData("TeleOp heading", localizer.getRotation() );
        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();
    }
}