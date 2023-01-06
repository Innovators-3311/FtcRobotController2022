package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AnglePIDControl;
import org.firstinspires.ftc.teamcode.util.localizers.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;

public class PacManTurnToPos {
private double goalHeading = 0;
private double turnSpeed   = 0;
private Localizer localizer;
private MecanumDriveBase mecanumDriveBase;
private AnglePIDControl angleControl;
CompassSensor                compass;




    public PacManTurnToPos(Localizer localizer, MecanumDriveBase mecanumDriveBase) {
        this.localizer = localizer;
        this.mecanumDriveBase = mecanumDriveBase;
        this.angleControl = new AnglePIDControl(.05, 0, .003,360);
        this.angleControl.setTargetValue(0);
    }

    public void handlePacMan(Gamepad gamepad, Telemetry telemetry) {
        //makes heading easier for me.
        double heading =  localizer.getRotation();
        double angleError = 0;

        if(Math.abs(gamepad.right_stick_y) < .1 && Math.abs(gamepad.right_stick_x) < .1 &&
                Math.abs(gamepad.left_stick_y) < .1 && Math.abs(gamepad.left_stick_x) < .1) {

            boolean dpad_used = false;
            if (gamepad.dpad_down) {
                angleError = smartAngleError(heading, 180);
                mecanumDriveBase.driveMotors(0, -angleControl.update(angleError), 0, 1);
                dpad_used = true;
            }
            if (gamepad.dpad_up) {
                angleError = smartAngleError(heading, 0);
                mecanumDriveBase.driveMotors(0, -angleControl.update(angleError), 0, 1);
                dpad_used = true;
            }
            if (gamepad.dpad_left) {
                angleError = smartAngleError(heading, 270);
                mecanumDriveBase.driveMotors(0, -angleControl.update(angleError), 0, 1);
                dpad_used = true;
            }
            if (gamepad.dpad_right) {
                angleError = smartAngleError(heading, 90);
                mecanumDriveBase.driveMotors(0, -angleControl.update(angleError), 0, 1);
                dpad_used = true;
            }
            if (dpad_used && (Math.abs(angleError) < 5)){
                mecanumDriveBase.driveMotors(1,0,0,1);
                telemetry.addData("forward", 0);
            }
            telemetry.addData("angleError", angleError);

        }

    }
    /**
     * Computes an Angle Error calculation, accounting for wrap arounds.
     *
     * @param a angle a
     * @param b angle b
     * @return angle error (a-b, as close to zero as possible)
     */
    public double smartAngleError(double a, double b){
        double diff = a - b;
        return diff - Math.round(diff/360.0) * 360.0;
    }
}
