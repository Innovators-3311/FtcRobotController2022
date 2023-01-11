package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;

public class DriveToPos {
    private final CombinedLocalizer localizer;
    private MecanumDriveBase mecanumDriveBase;
    private double xTarget = 0;
    private double yTarget = 0;
    private double rotationTarget = 0;
    private double xStart = 0;
    private double yStart = 0;
    private double rotationStart = 0;
    private final double safeDistance = 7;
    public double maxSpeedFactor = 1.0;

    /**
     * Constructor for Relative Drive Controller
     *
     * @param localizer the localizer that knows where we are.
     * @param mecanumDriveBase The drive base to command.
     */
    public DriveToPos(CombinedLocalizer localizer, MecanumDriveBase mecanumDriveBase) {
        this.localizer = localizer;
        this.mecanumDriveBase = mecanumDriveBase;
    }

    /**
     * Save start position.
     *
     * @param x Save x start position
     * @param y Save y start position
     * @param rotation Save rotation start position
     */
    public void setStart(double x, double y, double rotation){
        xStart = x;
        yStart = y;
        rotationStart = rotation;
    }

    /**
     * Move relative to the robot's current position in the field frame of reference.
     *
     * @param x Inches to move in the Field's X frame.
     * @param y Inches to move in the Field's Y frame.
     * @param rotation degrees to rotate (using compass rotation)
     */
    public void setTarget(double x, double y, double rotation){
        xTarget = x;
        yTarget = y;
        rotationTarget = rotation;
    }

    /**
     * Drive to a field's absolute position, using the localizer uncertainty as a threshold.
     *
     * @return Final error distance (in inches)
     */
    public void driveToPosition(){
        double DX = xTarget - localizer.x;
        double DY = yTarget - localizer.y;
        double distanceToTarget = distanceToTarget();

        double[] driveVector = localizer.fieldToRobotFrame(DX/distanceToTarget,
                DY/distanceToTarget);

        // Compute the turn that minimizes wrap-arounds.
        // Note that this is a heading change, so rotation and target are backwards
        // TODO: Fix handling heading change! -1 < turn < 1
        double turn = localizer.smartAngleError(localizer.getRotation(), rotationTarget);
        double speedFactor = 1;

        // https://www.desmos.com/calculator/ln1qieke73
        if (distanceToTarget < safeDistance) {
            speedFactor = maxSpeedFactor/safeDistance*distanceToTarget;
        }
        if(distanceToTarget<localizer.positionUncertainty){
            speedFactor = 0;
        }



        // Drive the motors!
        mecanumDriveBase.driveMotors(driveVector[0], turn, driveVector[1], speedFactor);

    }

    /**
     * Compute the distance from the robot to the target
     *
     * @return Distance to the target (inches)
     */
    public double distanceToTarget(){
        double DX = xTarget - localizer.x;
        double DY = yTarget - localizer.y;
        return Math.sqrt(DX*DX + DY*DY);
    }

}
