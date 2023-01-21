package org.firstinspires.ftc.teamcode.util.odometry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class JunctionAligner {
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;

    public double xCorrection;
    public double zCorrection;

    public void AlignRobot(){
        if(gamepad1.a){
            if (leftSensor.getDistance(DistanceUnit.CM) <= 25 && rightSensor.getDistance(DistanceUnit.CM) <= 25){
                //1 + 1 = 2 -Auggie
                xCorrection = (-leftSensor.getDistance(DistanceUnit.CM) + rightSensor.getDistance(DistanceUnit.CM)) / 2;
                if((leftSensor.getDistance(DistanceUnit.CM) + rightSensor.getDistance(DistanceUnit.CM)) / 2 > 9){
                    zCorrection = (leftSensor.getDistance(DistanceUnit.CM) + rightSensor.getDistance(DistanceUnit.CM)) / 4;
                }
            }
        }
    }
}
