package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Zone Parking", group = "Autonomous")

public class ParkAtZone extends LinearOpMode
{
    private DetectConeSleeve coneSleeveDetector = null;
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private int location = 0;
    private double max = 0.0;

//    public ParkAtZone(DetectConeSleeve coneSleeveDetector)
//    {
//        this.coneSleeveDetector = coneSleeveDetector;
//    }

    @Override
    public void runOpMode()
    {
        waitForStart();

//        coneSleeveDetector = new DetectConeSleeve();
//
//        telemetry.addData("Number1", "%d", 1);
//        telemetry.update();
//        while (opModeIsActive())
//        {
//            telemetry.addData("Number2", "%d", 2);
//            telemetry.update();
//
//            location = coneSleeveDetector.ConeDetection();
//            telemetry.addData("Number3", "%d", location);
//            telemetry.update();
//            sleep(1000);
//        }
//        telemetry.update();
//        telemetry.addData("Number4", "%d", 3);
//        sleep(1000);

    }
}
