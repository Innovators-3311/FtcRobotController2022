package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.DriveForTest;
import org.firstinspires.ftc.teamcode.util.localizers.IntegratedLocalizerIMU;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;
import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BackUpCode", group="!")
public class BasicTeleOp extends OpMode

{
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private DriveForTest driveForTest;

    public void init()
    {
        telemetry.addData("Status", "Initialized");
        towerController = new TowerController(hardwareMap, telemetry);
        driveForTest = new DriveForTest(hardwareMap);
    }
    @Override
    public void loop()
    {
        towerController.handleGamepad(gamepad2, telemetry);
        driveForTest.drive(gamepad1);
        telemetry.update();
    }
}