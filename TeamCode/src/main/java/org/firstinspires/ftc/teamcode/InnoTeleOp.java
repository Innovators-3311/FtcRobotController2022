package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;
import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;
import org.firstinspires.ftc.teamcode.util.localizers.IntegratedLocalizerIMU;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStub1", group="!")
public class InnoTeleOp extends OpMode

{
    private IntegratedLocalizerIMU localizer = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private CameraInitSingleton cameraInitSingleton;
    private WebcamName webcam;

    public void init() {
        cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        webcam = cameraInitSingleton.getWebcam();
        telemetry.addData("Status", "Initialized");
        localizer = new IntegratedLocalizerIMU(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
        towerController = new TowerController(hardwareMap, telemetry);
//        localizer = new LocalizerIMU(hardwareMap);
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
//        towerController.handleUBar();
        towerController.handleGamepad(gamepad2, telemetry);
//        towerController.handleScrew();
//        towerController.handleIntake();
//        towerController.handleGamepad(gamepad2);
        towerController.handleGamepad(gamepad2, telemetry);

        mecanumDriveBase.driveBaseTelemetry(telemetry);
        telemetry.addData("TeleOp heading", localizer.getRotation() );
        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.addData("", "lf = " + mecanumDriveBase.lf.getCurrentPosition());
        telemetry.addData("", "rf = " + mecanumDriveBase.rf.getCurrentPosition());
        telemetry.addData("", "lb = " + mecanumDriveBase.lb.getCurrentPosition());

//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();
    }
}