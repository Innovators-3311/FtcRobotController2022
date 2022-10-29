package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Cone Detector2", group = "Autonomous")
@Disabled
public class DetectCone extends LinearOpMode
{

    @Override
    public void runOpMode() throws  InterruptedException
    {
        SleeveDetector test = new SleeveDetector();
        waitForStart();

        InitVuforia initVuforia = new InitVuforia();
        initVuforia.initVuforiaMethod(hardwareMap);

        InitTensorFlow initTensorFlow = new InitTensorFlow();
        initTensorFlow.initTfod(hardwareMap);

        while (opModeIsActive())
        {
            test.detectSleeve(hardwareMap, telemetry);
        }
    }
}