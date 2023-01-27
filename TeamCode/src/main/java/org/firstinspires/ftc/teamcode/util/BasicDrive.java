package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

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
