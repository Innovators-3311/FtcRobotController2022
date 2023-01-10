package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer = new CombinedLocalizer(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
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