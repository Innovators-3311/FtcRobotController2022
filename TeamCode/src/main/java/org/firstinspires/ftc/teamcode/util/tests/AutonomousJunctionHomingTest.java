package org.firstinspires.ftc.teamcode.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.enums.JunctionType;
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

        relativeDriveController = new RelativeDriveController(mecanumDriveBase, odoPods);
        junctionHomingController = new JunctionHomingController(telemetry, mecanumDriveBase, hardwareMap, relativeDriveController);

        telemetry.addData("Hit", "start when ready", "");
        telemetry.update();

        // Waits till start button is pressed
        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.seconds();
        runtime.startTime();

        RobotLog.ii("AutonomousJunctionHomingTest", "Searching for a low junction.");
        boolean success = junctionHomingController.alignToPoleLeft(JunctionType.LOW);

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


