package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.ConeDetection;

@Autonomous(name = "Test cam", group = "")
public class TestClass extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        ConeDetection coneDetection = new ConeDetection();

        coneDetection.detector(telemetry, hardwareMap);

        waitForStart();
        stop();
    }
}
