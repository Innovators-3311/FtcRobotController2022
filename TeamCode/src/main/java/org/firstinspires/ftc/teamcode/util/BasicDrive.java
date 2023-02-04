package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Basic Drive", group = "basic drive")
@Disabled
public class BasicDrive extends OpMode
{
    private MecanumDriveBase mecanumDriveBase;
    private Gamepad gamepad1;

    @Override
    public void init()
    {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        telemetry.addData("!", "Jit start");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        mecanumDriveBase.gamepadController(gamepad1);
    }

}
