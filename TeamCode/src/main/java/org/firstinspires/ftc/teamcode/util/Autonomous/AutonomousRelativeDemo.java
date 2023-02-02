package org.firstinspires.ftc.teamcode.util.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.controllers.RelativeDriveController;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousRelativeConcept", group = "autonomous")
@Disabled
public class AutonomousRelativeDemo extends LinearOpMode
{

    private StateServer stateServer;
    private boolean blueSide;

    @Override
    public void runOpMode()
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

        RobotLog.ii("AutonomousRelativeDemo", "Driving forward 24 inches");
        RelativeDriveController waypoint1 = new RelativeDriveController(mecanumDriveBase, odoPods);

        // Drive foward and left one tile.
        waypoint1.setRelativeTarget(24, 0, 0);
        while (waypoint1.targetDistance() > 0.1) {
            waypoint1.handleRelativeDrive();
        }
        mecanumDriveBase.driveMotors(0.0,0.0,0.0, 0.0);


        RobotLog.ii("AutonomousRelativeDemo", "Strafing left 24 inches");
        RelativeDriveController waypoint2 = new RelativeDriveController(mecanumDriveBase, odoPods);
        // Drive foward and left one tile.
        waypoint2.setRelativeTarget(0, 24, 0);
        while (waypoint2.targetDistance() > 0.1) {
            waypoint2.handleRelativeDrive();
        }

        RobotLog.ii("AutonomousRelativeDemo", "Rotating 90 degrees to the right.");
        RelativeDriveController waypoint3 = new RelativeDriveController(mecanumDriveBase, odoPods);
        // Drive foward and left one tile.
        waypoint3.setRelativeTarget(0, 0, 90);
        while (waypoint3.targetHeadingError() > 1) {
            waypoint3.handleRelativeDrive();
        }


        stop();
    }

}


