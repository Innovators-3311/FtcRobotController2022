package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name = "Cone sleeve parking", group = "Autonomous")
public class DetectConeSleeve extends LinearOpMode
{
    private static int coneNumber = 10;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS =
            {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY = "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";

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

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        initVuforia();
        initTfod();
//        ConeDetection();
//    }
//
//    public void ConeDetection()
//    {


        int count = 0;

        if (tfod != null) {
            tfod.activate();

            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
//        telemetry.addData("String", "%s", "DetectConeSleeveMethode...");
//        telemetry.update();
//        waitForStart();

        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                if (tfod != null)
                {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null)
                    {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions)
                        {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

//                            telemetry.addData(""," ");
//                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
//                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
//                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);


                            String object = recognition.getLabel();
                            telemetry.addData("Label", "%s", object);

                            switch (object)
                            {
                                case "1 Bolt":
                                {
                                    this.coneNumber = 1;
                                    break;
                                }

                                case "2 Bulb":
                                {
                                    this.coneNumber = 2;
                                    break;
                                }

                                case "3 Panel":
                                {
                                    this.coneNumber = 3;
                                    break;
                                }

                                default:
                                {
                                    this.coneNumber = -1;
                                    break;
                                }
                            }

                            count++;
                            telemetry.addData("print", "%s", "print");
                            telemetry.update();
                            telemetry.addData("number", "%d", this.coneNumber);
                            telemetry.update();
                        }
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addData("false2", "%s", "reconation null");
                        telemetry.update();
                    }
                }
                else
                {
                    telemetry.addData("false1", "%s", "tfod null");
                    telemetry.update();
                }

            }
        }
        //return coneNumber;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia()
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
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}

