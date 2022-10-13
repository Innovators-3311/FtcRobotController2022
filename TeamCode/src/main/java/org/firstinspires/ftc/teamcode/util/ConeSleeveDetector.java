package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class ConeSleeveDetector
{
    private static int coneNumber = -1;

    public VuforiaInit vuforiaInit;
    public TFlowInit tFlowInit;

    public ConeSleeveDetector(HardwareMap hardwareMap)
    {
//        vuforiaInit = VuforiaInit.getInstance(hardwareMap);
        //vuforiaInit = new VuforiaInit(hardwareMap);
        tFlowInit = new TFlowInit(hardwareMap, VuforiaInit.getInstance(hardwareMap).getVuforia());
    }

    public void activateTfod()
    {
        if (tFlowInit.getTfod() != null) {
            tFlowInit.getTfod().activate();

            tFlowInit.getTfod().setZoom(1.0, 16.0/9.0);
        }
    }

    public void detectSleeve(Telemetry telemetry)
    {

        telemetry.addData("String", "%s", "Vuforia and tensor flow initated");
        telemetry.update();

//        if (tFlowInit.getTfod() != null)
//        {
//            tFlowInit.getTfod().activate();
//
//
//            tFlowInit.getTfod().setZoom(1.0, 16.0/9.0);
//        }
        boolean flag = true;
        boolean flag2 = true;
        while (flag)
        {
            if (tFlowInit.getTfod() != null)
            {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tFlowInit.getTfod().getUpdatedRecognitions();
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

