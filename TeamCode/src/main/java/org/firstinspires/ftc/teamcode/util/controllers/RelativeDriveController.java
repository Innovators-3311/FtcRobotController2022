package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.SimplePIDControl;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

public class RelativeDriveController {
    private final OdometryPodsSensor odometryPodsSensor;
    private final MecanumDriveBase mecanumDriveBase;

    public double speedFactor = 0.5;

    private double forward = 0.0;
    private double strafe = 0.0;
    private double heading = 0.0;

    private final double initialForward;
    private final double initialStrafe;
    private final double initialHeading;

    private final SimplePIDControl forwardControl;
    private final SimplePIDControl strafeControl;
    private final SimplePIDControl headingControl;

    /**
     * Start drive relative.
     *
     * @param mecanumDriveBase the drive base we're using.
     * @param odometryPodsSensor the odometry pods sensor we're using.
     */
    public RelativeDriveController(MecanumDriveBase mecanumDriveBase, OdometryPodsSensor odometryPodsSensor) {
        this.mecanumDriveBase = mecanumDriveBase;
        this.odometryPodsSensor = odometryPodsSensor;

        double[] state = odometryPodsSensor.getState();

        initialForward = state[0];
        initialStrafe = state[1];
        initialHeading = state[2];

        forwardControl = new SimplePIDControl(0.2, 0.0, 0.2);
        strafeControl = new SimplePIDControl(0.2, 0.0, 0.2);
        headingControl = new SimplePIDControl(0.1, 0.0, 0.2);
    }

    /**
     * Use this function to set the relative target drive location.
     *
     * @param forward the distance to go forward (negative is backward).
     * @param strafe the distance to strafe left (negative is right).
     * @param heading the degrees to turn right (negative is left).
     */
    public void setTarget(double forward, double strafe, double heading){
        this.forwardControl.targetValue = forward;
        this.strafeControl.targetValue = strafe;
        this.headingControl.targetValue = heading;
    }

    /**
     * Call this to drive the robot to the target you've assigned.
     *
     * @return distance to the intended relative target.
     */
    public double handleRelativeDrive(){
        double[] state = odometryPodsSensor.getState();

        double drive = forwardControl.update(state[0]);
        double turn = headingControl.update(state[2]);
        double strafe = strafeControl.update(state[1]);
        RobotLog.ii("RelativeDriveController",
                "x %.2f, y %.2f, rot %.2f  /  drive: %.2f strafe: %.2f turn: %.2f",
                state[0], state[1], state[2],
                drive, strafe, turn);
        this.mecanumDriveBase.driveMotors(drive, turn, strafe, speedFactor);

        double driveErr = forwardControl.targetValue - state[0];
        double strafeErr = strafeControl.targetValue - state[1];

        // Return the distance to the intended target.
        return Math.sqrt(driveErr * driveErr  + strafeErr * strafeErr);
    }
}
