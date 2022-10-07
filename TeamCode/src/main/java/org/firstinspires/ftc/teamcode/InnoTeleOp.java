package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Localizer;


@TeleOp(name="Basic drive", group="Basic drive")
public class InnoTeleOp extends OpMode
{
    private HardwareMap hardwareMap;

    private Localizer localizer = null;
    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer = new Localizer(hardwareMap);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        telemetry.update();
    }
}
