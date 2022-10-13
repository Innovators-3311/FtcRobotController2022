package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "drive to zone", group = "Autonomous")
public class DriveToZone extends LinearOpMode
{

    @Override
    public void runOpMode() throws  InterruptedException
    {
        ConeDetector coneDetector = new ConeDetector();
        waitForStart();

        while (opModeIsActive())
        {

        }
    }
}
