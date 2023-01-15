package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;

@TeleOp(name="MainCode", group="!")
public class TeleOpStub extends OpMode
{
    private CombinedLocalizer localizer        = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;

    public void init() {
        telemetry.addData("Status", "Initialized");
        RobotLog.ii("OpModeInit", "Constructing CombinedLocalizer");
        localizer = new CombinedLocalizer(hardwareMap);
        RobotLog.ii("OpModeInit", "Constructing MecanumDriveBase");
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
//        RobotLog.ii("OpModeInit", "Constructing TowerController");
//        towerController = new TowerController(hardwareMap, telemetry);
        RobotLog.ii("OpModeInit", "Constructing PacMan");
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
//        towerController.handleGamepad(gamepad2, telemetry);
//        telemetry.addData("TeleOp heading", localizer.getHeading());
//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.addData("", "lf = " + mecanumDriveBase.lf.getCurrentPosition());
        telemetry.addData("", "rf = " + mecanumDriveBase.rf.getCurrentPosition());
        telemetry.addData("", "lb = " + mecanumDriveBase.lb.getCurrentPosition());
        telemetry.addData("TeleOp heading", localizer.getRotation() );
        telemetry.update();
    }
}
