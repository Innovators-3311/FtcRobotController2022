package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamDetection {
    public boolean redTeam;
    public boolean blueTeam;
    public boolean rightSide;

    /**
     * Returns what team you are on from the switch.
     * requires a switch configured as "blueSwitch"
     * Blue is true, and Red is false
     *
     * @param hardwareMap OpMode hardwareMap
     *
     */
    public TeamDetection(HardwareMap hardwareMap){

        //find value of switch
        DigitalChannel blueSwitch = hardwareMap.get(DigitalChannel.class, "Switch");
        blueSwitch.setMode(DigitalChannel.Mode.INPUT);

//        DigitalChannel sideSwitch = hardwareMap.get(DigitalChannel.class, "side");
//        blueSwitch.setMode(DigitalChannel.Mode.INPUT);

        //set values according to the switches
        blueTeam = blueSwitch.getState();
        redTeam = !blueTeam;
//        rightSide = sideSwitch.getState();
    }

    /**
     * Broadcasts variables:
     * blueTeam (true/false)
     * redTeam (true/false)
     *
     * @param telemetry OpMode telemetry
     */
    public boolean showTeam(Telemetry telemetry) {

        //let the driver know what it detected
        if(redTeam)
        {
            return false;
        }

        if(blueTeam)
        {
            return true;
        }

        return false;
    }
}
//boom! now we know what team we're on!