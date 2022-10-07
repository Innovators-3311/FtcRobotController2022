package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Encoder drive", group = "Autonomous")
public class DriveToZone extends LinearOpMode
{
    private EncoderMethod encoderMethod;
    private HardwareMap hardwareMap;

    @Override
    public void runOpMode() throws  InterruptedException
    {
        Localizer localizer = new Localizer(hardwareMap);
        waitForStart();
//        encoderMethod.drive(encoderMethod.getCOUNTS_PER_INCH(), encoderMethod.getCOUNTS_PER_INCH(),encoderMethod.getCOUNTS_PER_INCH(), encoderMethod.getCOUNTS_PER_INCH(), encoderMethod.getSpeed());
        while (opModeIsActive())
        {
            encoderMethod.drive(encoderMethod.getCOUNTS_PER_INCH() * 8, encoderMethod.getCOUNTS_PER_INCH(), encoderMethod.getCOUNTS_PER_INCH(), encoderMethod.getCOUNTS_PER_INCH(), 0.25);
            Thread.sleep(1000);
            encoderMethod.drive(-encoderMethod.getCOUNTS_PER_INCH(), -encoderMethod.getCOUNTS_PER_INCH(), -encoderMethod.getCOUNTS_PER_INCH(), -encoderMethod.getCOUNTS_PER_INCH(), 0.50);
            Thread.sleep(1000);
            encoderMethod.drive(-1000, 1000, 1000, -1000, 100);
        }
    }
}
