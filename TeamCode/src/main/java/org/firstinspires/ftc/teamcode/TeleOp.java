package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.DriveForTest;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BackUp", group="Basic drive")
@Disabled
public class TeleOp extends OpMode

{
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
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