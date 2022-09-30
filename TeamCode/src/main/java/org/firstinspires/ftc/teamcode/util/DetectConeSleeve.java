package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

public class DetectConeSleeve
{
    private static int coneNumber = 10;

    private Telemetry telemetry;

    private VuforiaInit vuforiaInit;
    private TFlowInit tFlowInit;


    public void detectSleeve()
    {
        vuforiaInit = new VuforiaInit();
        vuforiaInit.initVuforia();
        tFlowInit = new TFlowInit();

        telemetry.addData("String", "%s", "Vuforia and tensor flow intiated");
        telemetry.update();

        if (tFlowInit.getTfod() != null)
        {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tFlowInit.getTfod().getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {

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
                    telemetry.addData("Detected: ", "%d", coneNumber);
                }
            }
        }
    }
}

