package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;

@TeleOp(name="TeleOpStub", group="Basic drive")
public class TeleOpStub extends OpMode

{
    private CombinedLocalizer localizer        = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;

    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer = new CombinedLocalizer(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        towerController = new TowerController(hardwareMap, telemetry);
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
        towerController.handleGamepad(gamepad2, telemetry);
//        telemetry.addData("TeleOp heading", localizer.getHeading());
//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.addData("", "lf = " + mecanumDriveBase.lf.getCurrentPosition());
        telemetry.addData("", "rf = " + mecanumDriveBase.rf.getCurrentPosition());
        telemetry.addData("", "lb = " + mecanumDriveBase.lb.getCurrentPosition());
        telemetry.addData("TeleOp heading", localizer.getRotation() );
        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();
    }
}
