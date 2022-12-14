package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.FieldObstacle;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;

public class BabyModeController {
    private MecanumDriveBase mecanumDriveBase ;
    private CombinedLocalizer combinedLocalizer;
    private static final double MIN_SPEED_FACTOR = 0.5;
    private FieldObstacle[] fieldObstacles;

    /* Needs a drivebase to control
     */
    public BabyModeController(MecanumDriveBase mecanumDriveBase, CombinedLocalizer combinedLocalizer) {
        this.mecanumDriveBase = mecanumDriveBase;
        this.combinedLocalizer = combinedLocalizer;
        createFieldObstacles();
    }

    public void createFieldObstacles(){
        int i = 0;

        for (int ndx=-2; ndx <= 2; ndx++){
            for(int ndy=-2; ndy <= 2; ndy++) {
                fieldObstacles[i++] = new FieldObstacle(ndx * 24, ndy * 24);
            }
        }
    }

    public double calculateSpeedFactor(){
        double speedFactor = 1.0;

        for (FieldObstacle o: fieldObstacles){
            // TODO: Should we use the commanded speed towards obstacle instead?
            double[] sto = o.speedTowardsObstacle(combinedLocalizer.x, combinedLocalizer.y,
                        combinedLocalizer.xVelocity, combinedLocalizer.yVelocity);
            double dist = sto[0];
            double rate = sto[1];

            // If we're close and moving towards the obstacle . . .
            if((dist<12) && (rate > 0)){
                // distance / rate os the time until we hit the obstacle (in seconds).
                // SF for this pole is the number of seconds until we hit the pole, but never less
                // than MIN_SPEED_FACTOR
                double thisSpeedFactor = Math.max(dist/rate, MIN_SPEED_FACTOR);

                // the final speed factor should be the smallest obstacle speed factor we find.
                speedFactor = Math.min(speedFactor, thisSpeedFactor);
            }
        }
        return speedFactor;
    }

    /**
     * Rotates a vector by an angle.
     *
     * @param vec [forward, strafe]
     * @param angle (degrees)
     * @return the rotated vector
     */
    public double [] rotateVector(double [] vec, double angle){
        double os = vec[0];
        double of = vec[1];
        double sin = Math.sin(angle * (Math.PI / 180.0));
        double cos = Math.cos(angle * (Math.PI / 180.0));
        double[] retval = {-sin * of + cos * os, cos * of  + sin * os};
        return retval;
    }

    /**
     * Rotate the joystick controls from field coordinates to robot forward and strafe. Returns
     * rotated forward and strafe
     *
     * @param strafe The joystick strafe command (x)
     * @param forward The joystick forward command (y)
     * @return [rotated_strafe (x') , rotated_forward (y')]
     */
    public double [] rotateControls(double strafe, double forward){
        double driveAngleOffSet = combinedLocalizer.heading;
        double [] vec = {strafe, forward};
        return rotateVector(vec, driveAngleOffSet);
    }

    public void handleGamepad(Gamepad gamepad){
        double drive = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;

        // Final Speed Factor is computed as the calculatedSpeedFactor plus the right trigger, which
        // can override the speed reduction that the calculated factor weould otherwise cause.
        double computedSF = calculateSpeedFactor();
        double speedFactor = Math.min(computedSF + .5 * gamepad.right_trigger, 1);

        mecanumDriveBase.driveMotors(drive, turn, strafe, speedFactor);
    }

}

