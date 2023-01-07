package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer          = new CombinedLocalizer(hardwareMap);
        mecanumDriveBase   = new MecanumDriveBase(hardwareMap);
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