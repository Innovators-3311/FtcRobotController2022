package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraInitSingleton;
import org.firstinspires.ftc.teamcode.util.controllers.JunctionHomingController;
import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;
import org.firstinspires.ftc.teamcode.util.controllers.RelativeDriveController;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

@TeleOp(name="MainCode", group="!")

public class CompTeleOp extends OpMode
{
//    private CombinedLocalizer localizer        = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private JunctionHomingController junctionHoming;
    private WebcamName webcam;
    private CameraInitSingleton cameraInitSingleton;

    public void init() {
        cameraInitSingleton = new CameraInitSingleton(hardwareMap);
        webcam = cameraInitSingleton.getWebcam();
        telemetry.addData("Status", "Initialized");
//        localizer = new CombinedLocalizer(hardwareMap, webcam);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        towerController = new TowerController(hardwareMap, telemetry);
        OdometryPodsSensor odoPods = new OdometryPodsSensor(hardwareMap);
        RelativeDriveController relativeDrive = new RelativeDriveController(mecanumDriveBase, odoPods);
        junctionHoming = new JunctionHomingController(telemetry, mecanumDriveBase, hardwareMap, relativeDrive);
//        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
//        localizer.displayTelemetry(telemetry);
//        localizer.handleTracking();
        towerController.handleGamepad(gamepad2, telemetry);
        junctionHoming.handleGamepad(gamepad1,gamepad2);
        // If we're not homing, see what the driver wants.
        if (junctionHoming.aligningState == JunctionHomingController.AligningState.IDLE) {
            mecanumDriveBase.gamepadController(gamepad1);
            mecanumDriveBase.driveBaseTelemetry(telemetry);
        }

        //        telemetry.addData("TeleOp heading", localizer.getHeading());
//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.addData("", "lf = " + mecanumDriveBase.lf.getCurrentPosition());
        telemetry.addData("", "rf = " + mecanumDriveBase.rf.getCurrentPosition());
        telemetry.addData("", "lb = " + mecanumDriveBase.lb.getCurrentPosition());
//        telemetry.addData("TeleOp heading", localizer.getRotation() );
        telemetry.update();
    }
}
