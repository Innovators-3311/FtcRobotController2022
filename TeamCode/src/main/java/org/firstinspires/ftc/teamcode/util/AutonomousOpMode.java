package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.Localizer;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Sleeve Detection", group = "Autonomous")
public class AutonomousOpMode extends LinearOpMode
{
    private DetectConeSleeve detectConeSleeve;
    private Localizer localizer = null;

    @Override
    public void runOpMode()
    {
        localizer = new Localizer(hardwareMap);
        detectConeSleeve = new DetectConeSleeve();
        waitForStart();

        while (opModeIsActive())
        {
            detectConeSleeve.detectSleeve();
            localizer.displayTelemetry(telemetry);
            localizer.handleTracking();
            telemetry.update();
        }
    }
}
