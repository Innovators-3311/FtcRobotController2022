package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class VuforiaInit
{
    static VuforiaInit mVuforiaInit = null;

    private static final String VUFORIA_KEY = "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";

    public static VuforiaLocalizer vuforia;

    private static HardwareMap hardwareMap = null;

    private VuforiaInit(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = this.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //copied from localizer
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraName = webcamName; //Indicates which camera to use.
        parameters.useExtendedTracking = false; // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.


        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //this.vuforia.loadTrackablesFromFile("PowerPlay"); // loads images
    }

    public static VuforiaInit getInstance(HardwareMap hardwareMap)
    {

        if (mVuforiaInit == null)
        {
            mVuforiaInit = new VuforiaInit(hardwareMap);
        }

        return mVuforiaInit;
    }

    public VuforiaLocalizer getVuforia()
    {
        return vuforia;
    }

    /*
    public static VuforiaLocalizer getVuforia()
    {
        if ()
        {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.cameraName = this.hardwareMap.get(WebcamName.class, "Webcam 1");
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }

        return VuforiaLocalizer
    }
    */


}
