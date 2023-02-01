package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;
import org.firstinspires.ftc.teamcode.util.controllers.JunctionHomingController;
import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;
import org.firstinspires.ftc.teamcode.util.controllers.RelativeDriveController;
import org.firstinspires.ftc.teamcode.util.enums.JunctionType;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.util.localizers.IntegratedLocalizerIMU;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

import java.security.KeyStore;

@TeleOp(name="Teleop Testing", group="zz-test")
public class TeleOpTesting extends OpMode

{
//    private CombinedLocalizer localizer = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private JunctionHomingController junctionHoming;

    public void init() {
        OdometryPodsSensor odoPods = new OdometryPodsSensor(hardwareMap);
        CameraInitSingleton cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        WebcamName webcam = cameraInitSingleton.getWebcam();
        telemetry.addData("Status", "Initialized");
//        localizer = new CombinedLocalizer(hardwareMap,webcam);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        //    private PacManTurnToPos pacMan;
        RelativeDriveController relativeDrive = new RelativeDriveController(mecanumDriveBase, odoPods);
        towerController = new TowerController(hardwareMap, telemetry);
        junctionHoming = new JunctionHomingController(telemetry,mecanumDriveBase,hardwareMap, relativeDrive);
//        localizer = new LocalizerIMU(hardwareMap);
//        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
//        localizer.displayTelemetry(telemetry);
//        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);

        // This overrides mecanumDriveBase commands.
        junctionHoming.handleGamepad(gamepad1, gamepad2);
//        towerController.handleUBar();
        towerController.handleGamepad(gamepad2, telemetry);
//        towerController.handleScrew();
//        towerController.handleIntake();
//        towerController.handleGamepad(gamepad2);
        towerController.handleGamepad(gamepad2, telemetry);

        mecanumDriveBase.driveBaseTelemetry(telemetry);
//        telemetry.addData("TeleOp heading", localizer.getRotation() );
//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.addData("", "lf = " + mecanumDriveBase.lf.getCurrentPosition());
        telemetry.addData("", "rf = " + mecanumDriveBase.rf.getCurrentPosition());
        telemetry.addData("", "lb = " + mecanumDriveBase.lb.getCurrentPosition());

//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();
    }
}