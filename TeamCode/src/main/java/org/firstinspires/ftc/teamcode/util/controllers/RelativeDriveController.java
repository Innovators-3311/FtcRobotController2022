package org.firstinspires.ftc.teamcode.util.controllers;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.SimplePIDControl;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;

public class RelativeDriveController {
    private final OdometryPodsSensor odometryPodsSensor;
    private final MecanumDriveBase mecanumDriveBase;

    public double speedFactor = 0.5;

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

        forwardControl = new SimplePIDControl(0.8, 0.1, 0.2);
        strafeControl  = new SimplePIDControl(0.8, 0.1, 0.2);
        headingControl = new SimplePIDControl(0.2, 0.1, 0.2);
    }

    /**
     * Use this function to set the relative target drive location.
     *
     * @param forward the distance to go forward (negative is backward).
     * @param strafe the distance to strafe left (negative is right).
     * @param heading the degrees to turn right (negative is left).
     */
    public void setTarget(double forward, double strafe, double heading){
        this.forwardControl.targetValue = initialForward + forward;
        this.strafeControl.targetValue = initialStrafe + strafe;
        this.headingControl.targetValue = initialHeading + heading;
    }

    /**
     * Compute the distance to the target
     *
     * @return distance to target location.
     */
    public double targetDistance() {
        double[] state = odometryPodsSensor.getState();
        double fwdErr = this.forwardControl.measuredError(state[0]);
        double strErr = this.strafeControl.measuredError(state[1]);

        return sqrt(fwdErr * fwdErr + strErr * strErr);
    }

    /**
     * Compute the error in heading compared to the target heading
     *
     * @return distance to target location.
     */
    public double targetHeadingError() {
        double[] state = odometryPodsSensor.getState();
        return this.headingControl.measuredError(state[2]);
    }

    /**
     * Call this to drive the robot to the target you've assigned.
     *
     */
    public void handleRelativeDrive(){
        double[] state = odometryPodsSensor.getState();

        double drive = forwardControl.update(state[0]);
        double strafe = strafeControl.update(state[1]);
        double turn = headingControl.update(state[2]);

        RobotLog.ii("RelativeDriveController",
                "x %.2f, y %.2f, rot %.2f  /  drive: %.2f strafe: %.2f turn: %.2f",
                state[0], state[1], state[2],
                drive, strafe, turn);
        this.mecanumDriveBase.driveMotors(drive, turn, strafe, speedFactor);

        double driveErr = forwardControl.targetValue - state[0];
        double strafeErr = strafeControl.targetValue - state[1];
    }
}
