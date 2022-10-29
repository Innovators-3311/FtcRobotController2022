package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class SleeveDetector
{

    private static final String[] LABELS =
            {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private int coneNumber = -1;

    public void detectSleeve(HardwareMap hardwareMap, Telemetry telemetry)
    {
        InitTensorFlow initTensorFlow = new InitTensorFlow();

        telemetry.addData("String", "%s", "Vuforia and tensor flow initated");
        telemetry.update();

        if (initTensorFlow.getTfod() != null)
        {
            initTensorFlow.getTfod().activate();
            telemetry.addData("", "&s", "activate");
            initTensorFlow.getTfod().setZoom(1.0, 16.0 / 9.0);
        }
        boolean flag = true;
        boolean flag2 = true;
        while (flag)
        {
            if (initTensorFlow.getTfod() != null)
            {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = initTensorFlow.getTfod().getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {

                    for (Recognition recognition : updatedRecognitions)
                    {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        String object = recognition.getLabel();

                        switch (object)
                        {
                            case "1 Bolt":
                            {
                                this.coneNumber = 1;
                                telemetry.addData("Cone:", "%d", coneNumber);
                                flag = false;
                                break;
                            }

                            case "2 Bulb":
                            {
                                this.coneNumber = 2;
                                telemetry.addData("Cone:", "%d", coneNumber);
                                flag = false;
                                break;
                            }

                            case "3 Panel":
                            {
                                this.coneNumber = 3;
                                telemetry.addData("Cone:", "%d", coneNumber);
                                flag = false;
                                break;
                            }

                            default:
                            {
                                this.coneNumber = -1;
                                telemetry.addData("Cone:", "%d", coneNumber);
                                break;
                            }
                        }
                        telemetry.addData("Detected: ", "%d", coneNumber);
                        telemetry.update();
                    }
                }

            }
        }
    }
}

/*
    public void initVuforia(HardwareMap hardwareMap)
    {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod( HardwareMap hardwareMap)
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
*/