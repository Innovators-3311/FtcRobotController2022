package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class VuforiaInit
{
    private static final String VUFORIA_KEY = "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";

    public VuforiaLocalizer vuforia;

    public VuforiaInit(HardwareMap hardwareMap)
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

}
