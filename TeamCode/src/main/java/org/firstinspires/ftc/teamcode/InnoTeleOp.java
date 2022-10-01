package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Localizer;


@TeleOp(name="TeleOpStub", group="Basic drive")
public class InnoTeleOp extends OpMode
{
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
