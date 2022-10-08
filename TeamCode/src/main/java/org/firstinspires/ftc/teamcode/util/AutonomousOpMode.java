package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Sleeve Detection", group = "Autonomous")
public class AutonomousOpMode extends LinearOpMode
{
    private ConeSleeveDetector detectConeSleeve;
    //private Localizer localizer;

    @Override
    public void runOpMode()
    {
     //  localizer = new Localizer(hardwareMap);
        detectConeSleeve = new ConeSleeveDetector(hardwareMap);

        // Start VuForia background process looking for vumarks in camera field of view. Activate
        // before waitForStart() allows you to see camera stream on DS at INIT wait. See DS menu.
        detectConeSleeve.activateTfod();

        // Code Works!!!!!!!!!!!!!!!!!!!!!!!!!!

        waitForStart();

        detectConeSleeve.detectSleeve(telemetry);



        while (opModeIsActive())
        {
//            telemetry.addData("Location", "%d", parkingLocation);
      //      localizer.displayTelemetry(telemetry);
        //g    localizer.handleTracking();
            telemetry.update();
        }
    }

}
