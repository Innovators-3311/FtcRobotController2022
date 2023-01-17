package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ConeDetection;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TeamDetection;
import org.firstinspires.ftc.teamcode.util.controllers.RelativeDriveController;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousRelativeConcept", group = "autonomous")
public class AutonomousRelativeDemo extends LinearOpMode
{

    private StateServer stateServer;

    private boolean blueSide;

    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDriveBase mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        OdometryPodsSensor odoPods = new OdometryPodsSensor(hardwareMap);

        telemetry.addData("Hit", "start when ready", "");
        telemetry.update();

        // Waits till start button is pressed
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        runtime.seconds();
        runtime.startTime();

        RelativeDriveController waypoint1 = new RelativeDriveController(mecanumDriveBase, odoPods);
        double distance = 100;
        // Drive foward and left one tile.
        waypoint1.setTarget(24, 24, 0);
        while (distance > 1) {
            distance = waypoint1.handleRelativeDrive();
            telemetry.addData("Distance to target:", distance);
            telemetry.update();
        }

    }

}


