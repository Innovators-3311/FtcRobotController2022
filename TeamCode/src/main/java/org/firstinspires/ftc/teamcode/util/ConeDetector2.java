package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name = "Cone Detector other", group = "Autonomous")
@Disabled
public class ConeDetector2 extends LinearOpMode
{

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/DefaultConeTest.tflite";

//    private static final String[] LABELS = {
//            "1 Zone1",
//            "2 Zone2",
//            "3 Zone3"
//    };

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

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

    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;

    private int coneNumber = -1;

    private final double COUNTS_PER_MOTOR_GOBUILDA202 = 384.5 ;    // eg: TETRIX Motor Encoder REV    = 1440
    private final double DRIVE_GEAR_REDUCTION = 3;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    public double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode()
    {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        initVuforia(hardwareMap);
        initTfod(hardwareMap);
        telemetry.addData("->", "%s", "Hit start or else...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive())
        {

            telemetry.update();

            if (tfod != null)
            {
                tfod.activate();
                tfod.setZoom(2.0, 16.0/9.0);

            }
            boolean flag = true;
            while (flag)
            {
//             telemetry.addData("", "%s", "while (flag)");
//             telemetry.update();
                if (tfod != null)
                {
//                 telemetry.addData("", "%s", "if (tfod != null)");
//                 telemetry.update();
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null)
                    {
//                     telemetry.addData("", "%s", "if (updatedRecognitions != null)");
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        telemetry.update();
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
                                    flag = false;
                                    break;
                                }
                            }
                            if (1 == coneNumber || 2 == coneNumber || 3 == coneNumber)
                            {
                                break;
                            }
                        }
                    }

                }
            }


            // Movement code
            drive(COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, COUNTS_PER_INCH * 18, 1.0);
            while (opModeIsActive() && lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())
            {
                idle();
            }
            sleep(500);
            switch (coneNumber)
            {
                case 1:
                    drive(-(COUNTS_PER_INCH * 23), COUNTS_PER_INCH * 23, COUNTS_PER_INCH * 23, -(COUNTS_PER_INCH * 23), 1.0);
                    while (opModeIsActive() && lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())
                    {
                        idle();
                    }
                    break;

                case 3:
                    drive(COUNTS_PER_INCH * 23, -(COUNTS_PER_INCH * 23), -(COUNTS_PER_INCH * 23), COUNTS_PER_INCH * 23, 1.0);
                    while (opModeIsActive() && lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())
                    {
                        idle();
                    }
                    break;

                default:
                    break;
            }

        }
    }

    // forward drive(1000, 1000, 1000, 1000, 0.25);
    // Turn drive(1000, -1000, 1000, -1000, 0.25);
    // strafe drive(1000, -1000, -1000, 1000, 0.25);

    public void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed)
    {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int leftFrontPos = 0;
        int rightFrontPos = 0;
        int leftBackPos = 0;
        int rightBackPos = 0;

        leftFrontPos += leftFrontTarget;
        rightFrontPos += rightFrontTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;

        lf.setTargetPosition(leftFrontPos);
        rf.setTargetPosition(rightFrontPos);
        lb.setTargetPosition(leftBackPos);
        rb.setTargetPosition(rightBackPos);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

    }

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
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS2);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
