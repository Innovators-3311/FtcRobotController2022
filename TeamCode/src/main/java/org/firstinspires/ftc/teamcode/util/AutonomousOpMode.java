package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Localizer;


@Autonomous(name = "Sleeve Detection", group = "Autonomous")
public class AutonomousOpMode extends LinearOpMode
{
    private DetectConeSleeve detectConeSleeve;
    private Localizer localizer = null;

    @Override
    public void runOpMode()
    {
        localizer = new Localizer(hardwareMap);
        detectConeSleeve = new DetectConeSleeve(hardwareMap);
        waitForStart();
        detectConeSleeve.detectSleeve(telemetry);
        while (opModeIsActive())
        {
            localizer.displayTelemetry(telemetry);
            localizer.handleTracking();
            telemetry.update();
        }
    }

}
