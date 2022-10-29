package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;


@TeleOp(name="Basic drive2", group="Basic drive")
@Disabled
public class InnoTeleOp extends OpMode
{
    MecanumDriveBase mecanumDriveBase;
    private Localizer localizer = null;

    public void init() {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        telemetry.addData("Status", "Initialized");
        localizer = new Localizer(hardwareMap);
        double max;

    }
    @Override
    public void loop() {

        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        telemetry.update();
    }
}
