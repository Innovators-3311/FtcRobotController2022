package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class CameraInitSingleton
{
    private static WebcamName webcam;

    public CameraInitSingleton(HardwareMap hardwareMap)
    {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public WebcamName getWebcam()
    {
        return webcam;
    }

































    //    private static final CameraInitSingleton instance = new CameraInitSingleton();

//    private CameraInitSingleton() {}
//
//    public static CameraInitSingleton getInstance(HardwareMap hardwareMap)
//    {
//        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        return instance;
//    }

//        private static final EagerInitializedSingleton instance = new EagerInitializedSingleton();
//
//        // private constructor to avoid client applications using the constructor
//        private EagerInitializedSingleton(){}
//
//        public static EagerInitializedSingleton getInstance() {
//            return instance;
//        }
//    }
}
