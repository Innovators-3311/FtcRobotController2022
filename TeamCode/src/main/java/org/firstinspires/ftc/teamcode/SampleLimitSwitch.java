package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "SampleLimitSwitch", group = "Linear OpMode")
public class SampleLimitSwitch extends LinearOpMode {
        TouchSensor limitSwitch;
        @Override
        public void runOpMode() {
            // Get the touch sensor and motor from hardwareMap
            limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

            // Wait for the play button to be pressed
            waitForStart();

            // Loop while the Op Mode is running
            while (opModeIsActive()) {
                if (limitSwitch.isPressed()) {
                    telemetry.addData("Status", "Limit Switch is Pressed");
                } else {
                    telemetry.addData("Status", "Limit Switch is not Pressed");
                }
                telemetry.update();
            }
        }
    }