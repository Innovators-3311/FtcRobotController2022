package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.FieldObstacle;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;

public class BabyModeController {
    private MecanumDriveBase mecanumDriveBase ;
    private CombinedLocalizer combinedLocalizer;

    private FieldObstacle[] fieldObstacles;

    /* Needs a drivebase to control
     */
    public BabyModeController(MecanumDriveBase mecanumDriveBase) {
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
            double[] sto = o.speedTowardsObstacle(combinedLocalizer.x, combinedLocalizer.y,
                        combinedLocalizer.xVelocity, combinedLocalizer.yVelocity);
            double dist = sto[0];
            double rate = sto[1];

            // If we're close and moving towards the obstacle . . .
            if((dist<12) && (rate > 0)){
                // speedFactor can be reduced if we're going to hit in less than a second,
                // but it doesn't reduce below 0.5.
                speedFactor = Math.min(speedFactor, Math.max(dist/rate, 0.5));
            }
        }
        return speedFactor;
    }

    public double [] rotate_controls(double forward, double strafe){
        double of = forward;
        double os = strafe;
        double driveAngleOffSet = combinedLocalizer.heading;

        double sin = Math.sin(driveAngleOffSet * (Math.PI / 180.0));
        double cos = Math.cos(driveAngleOffSet * (Math.PI / 180.0));
        double[] retval = {cos * of  + sin * os, -sin * of + cos * os};
        return retval;
    }

    public void handleGamepad(Gamepad gamepad){
        double drive = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;
        // TODO: Make this smarter!
        double speedFactor = .5 + .5 * gamepad.right_trigger;

        mecanumDriveBase.driveMotors(drive, turn, strafe, speedFactor);
    }

}

