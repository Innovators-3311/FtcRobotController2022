package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Zone Parking", group = "Autonomous")
@Disabled
public class ParkAtZone extends OpMode
{
    private ConeSleeveDetector coneSleeveDetector = null;
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private int location = 0;
    private double max = 0.0;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    //    public ParkAtZone(DetectConeSleeve coneSleeveDetector)
//    {
//        this.coneSleeveDetector = coneSleeveDetector;
//    }

}