package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Zone Parking", group = "Autonomous")
@Disabled
public class ParkAtZone extends OpMode
{
    private DetectConeSleeve coneSleeveDetector = null;
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