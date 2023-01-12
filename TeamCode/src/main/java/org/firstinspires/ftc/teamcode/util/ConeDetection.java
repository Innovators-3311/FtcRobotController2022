package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

public class ConeDetection
{

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
//    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/wheel_fish_eye.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

//    private static final String[] LABELS = {
//            "fish",
//            "eye",
//            "wheel"
//    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private int coneNumber = -1;

    public int detector(Telemetry telemetry, HardwareMap hardwareMap)
    {
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(2.5, 10.0/10.0);

            //tfod.setClippingMargins(400, 300, 50, 0);
            //tfod.setZoom(1, 1);
        }

        boolean flag = true;
        while (flag)
        {
//            telemetry.addData("", "%s", "while (flag)");
//            telemetry.update();
            if (tfod != null)
            {
//                telemetry.addData("", "%s", "if (tfod != null)");
//                telemetry.update();
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {
//                    telemetry.addData("", "%s", "if (updatedRecognitions != null)");
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    telemetry.update();
                    for (Recognition recognition : updatedRecognitions)
                    {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
//                        telemetry.addData("", " ");
//                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
//                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        String object = recognition.getLabel();
                        switch (object)
                        {
                            case "1 bolt":
                            {
                                this.coneNumber = 1;
                                flag = false;
                                break;
                            }
                            case "2 Bulb":
                            {
                                this.coneNumber = 2;
                                flag = false;
                                break;
                            }
                            case "3 Panel":
                            {
                                this.coneNumber = 3;
                                flag = false;
                                break;
                            }
                            default:
                            {
                                break;
                            }
                        }
                        if (1 == coneNumber || 2 == coneNumber || 3 == coneNumber)
                        {
                            return coneNumber;
                        }
                    }
                }
            }
        }
        return -1;
    }

    // forward drive(1000, 1000, 1000, 1000, 0.25);
    // Turn drive(1000, -1000, 1000, -1000, 0.25);
    // strafe drive(1000, -1000, -1000, 1000, 0.25);


    /**
     * Initialize the Vuforia localization engine.
     */

    private void initVuforia(HardwareMap hardwareMap)
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod( HardwareMap hardwareMap)
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 50;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

}