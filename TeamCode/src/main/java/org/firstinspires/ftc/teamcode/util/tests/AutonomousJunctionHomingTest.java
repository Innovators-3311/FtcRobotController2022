package org.firstinspires.ftc.teamcode.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.controllers.JunctionHomingController;
import org.firstinspires.ftc.teamcode.util.controllers.RelativeDriveController;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousHomingTest", group = "zz-test")
public class AutonomousJunctionHomingTest extends LinearOpMode
{

    private StateServer stateServer;
    private boolean blueSide;
    private JunctionHomingController junctionHomingController;
    private RelativeDriveController relativeDriveController;
    private MecanumDriveBase mecanumDriveBase;
    private static double ticksPerRotation = 384.5 ;
    private static double ticksPerInch = ticksPerRotation / (Math.PI * 4);

    @Override
    public void runOpMode()
    {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        OdometryPodsSensor odoPods = new OdometryPodsSensor(hardwareMap);

        junctionHomingController = new JunctionHomingController(telemetry, mecanumDriveBase, hardwareMap);
        relativeDriveController = new RelativeDriveController(mecanumDriveBase, odoPods);

        telemetry.addData("Hit", "start when ready", "");
        telemetry.update();

        // Waits till start button is pressed
        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.seconds();
        runtime.startTime();

//        driveDistance(-12);
//        junctionHomingController.rotate(-30, 1, false);
        RobotLog.ii("AutonomousJunctionHomingTest", "Searching for a cone.");

        junctionHomingController.rotate(90, 0.6, true);
        double forward = junctionHomingController.getDistance() - 3;
        driveDistance(forward);
        mecanumDriveBase.driveMotors(0, 0, 0, 1);
        junctionHomingController.rotate(-20, 0.4, false);
        junctionHomingController.rotate(40, 0.2, true);

        sleep(1);
        stop();
    }

    private void driveDistance(double forward) {
        double sign = forward / Math.abs(forward);
        double initial_position = mecanumDriveBase.rf.getCurrentPosition();

        mecanumDriveBase.driveMotors(sign, 0, 0, 0.6);
        while (Math.abs(mecanumDriveBase.rf.getCurrentPosition() - initial_position) < Math.abs(forward)*ticksPerInch){
            sleep(1);
        }

    }
}


