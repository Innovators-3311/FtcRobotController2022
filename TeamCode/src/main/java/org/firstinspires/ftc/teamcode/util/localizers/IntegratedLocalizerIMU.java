package org.firstinspires.ftc.teamcode.util.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.util.InternalIMUSensor;
import org.firstinspires.ftc.teamcode.util.localizers.Localizer;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class IntegratedLocalizerIMU implements Localizer {
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    private static final float loopSpeedHT = 0.1f;

    private static final String VUFORIA_KEY =
            "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";
    final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // FIXME
    final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // FIXME
    final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // FIXME
    private double y         = 0;
    private double x         = 0;
    private double z         = 0;
    private double yVelocity = 0;
    private double xVelocity = 0;
    private double heading   = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation            = null;
    private VuforiaLocalizer vuforia             = null;
    private VuforiaTrackables targets            = null;
    private InternalIMUSensor imu;
    private WebcamName webcamName                = null;
    private List<VuforiaTrackable> allTrackables = null;
    private boolean targetVisible                = false;
    private double lastT          = 0;
    private double headingOffSet  = 0;
    private double imuHeading = 0;
    private double vuforiaHeading = 0;

    private DataLogger dataLogger;

    public IntegratedLocalizerIMU(HardwareMap hardwareMap) {
        runtime.reset();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        imu = new InternalIMUSensor(hardwareMap);

        try {
            dataLogger = new DataLogger("/sdcard/FIRST/matchlogs/Match-1-TeleOpStub.csv");
            dataLogger.addHeaderLine("Time", "x", "y", "angle", "xRate", "yRate", "angleRate");
        } catch (IOException io) {

        }


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName; //Indicates which camera to use.
        parameters.useExtendedTracking = false; // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        vuforia = ClassFactory.getInstance().createVuforia(parameters); // Initiates Vuforia.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay"); // loads images
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        identifyTarget(0, "Red Audience Wall", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        targets.activate();
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */

    public void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }


    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Delta T", runtime.seconds() - lastT);
        telemetry.addData("x position", x);
        telemetry.addData("y position", y);
        telemetry.addData("z position", z);
        telemetry.addData("x velocity", xVelocity);
        telemetry.addData("y velocity", yVelocity);
        telemetry.addData("Heading", heading);
        telemetry.addData("Heading offset", headingOffSet);
//        telemetry.addData("heading gyro", integratedGyro.getHeading());
        telemetry.addData("heading Vuforia",vuforiaHeading);
        telemetry.addData("target visible?", targetVisible);

    }

    public void handleTracking() {
        imuHeading = imu.getHeading();
        this.heading = imuHeading + headingOffSet;
        if ((runtime.seconds() - lastT) < loopSpeedHT) {
            return;
        }
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            double lastX = x;
            double lastY = y;
            double t     = runtime.seconds();
            x = translation.get(0)/mmPerInch;
            y = translation.get(1)/mmPerInch;
            z = translation.get(2)/mmPerInch;
            xVelocity = (x-lastX) /(t-lastT);
            yVelocity = (y-lastY) /(t-lastT);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            vuforiaHeading = rotation.thirdAngle%360;
            this.heading = vuforiaHeading;
            headingOffSet  = vuforiaHeading - imu.getHeading();
            lastT = runtime.seconds();
        }
        else {
            heading = (imu.getHeading() + headingOffSet)%360;
        }
    }

    public double getHeading() {
        return heading;
    }

    //    public void gyroCalibrate(){
//        integratedGyro.calibrate();
//    }
}
