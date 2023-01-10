package org.firstinspires.ftc.teamcode.util.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.InternalIMUSensor;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryPodsSensor;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CombinedLocalizer implements Localizer {
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float fullField = 144 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    private static final float loopSpeedHT = 0.1f;

    private static final String VUFORIA_KEY =
            "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";
    final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // FIXME
    final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // FIXME
    final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // FIXME
    public double stateTime     = 0;
    public double y             = 0;
    public double x             = 0;
    public double z             = 0;
    public double yVelocity     = 0;
    public double xVelocity     = 0;
    public double rotation = 0;
    public double rotationRate = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation               = null;
    private VuforiaLocalizer vuforia                = null;
    private VuforiaTrackables targets               = null;
//    private GyroSensor gyro;
    private InternalIMUSensor imu;
    private OdometryPodsSensor odoPods;
    private WebcamName webcamName                  = null;
    private List<VuforiaTrackable> allTrackables   = null;
    VuforiaTrackable trackable;
    private boolean targetVisible                  = false;
    private boolean targetWasVisible                  = false;
    private StateServer stateServer;
    private double lastT                           =  0;
    private double headingOffSet                   =  0;
    private double gyroHeading                     =  0;
    public double positionUncertainty             = .5;
    static final double vuforiaPositionUncertainty = .2;
    private double headingUncertainty              = 5;
    private double vuforiaHeadingUncertainty       = 5;

    /**
     * Construct the combined localizer
     *
     * @param hardwareMap a hardware map.
     */
    public CombinedLocalizer(HardwareMap hardwareMap) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        gyro = hardwareMap.get(GyroSensor.class, "gyro");
        imu = new InternalIMUSensor(hardwareMap);
        try{
            stateServer = new StateServer();
        }catch (IOException e12141){
            RobotLog.ee("Localizer", "Error initializing NanoHTTPD StateServer");
        }
        odoPods = new OdometryPodsSensor(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName; //Indicates which camera to use.
        parameters.useExtendedTracking = false; // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        vuforia = ClassFactory.getInstance().createVuforia(parameters); // Initiates Vuforia.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay"); // loads images
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

//        identifyTarget(0, "Red Audience Wall", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
//        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
//        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
//        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        // WARNING: This is likely to be a source of error. Check z-rotations first.
        // Our reference frame is rotated -90 degrees about z (up) from the O.G. reference frame,
        // so we should be adding 90 degrees to the target z rotations.
        identifyTarget(0, "Red Audience Wall",
                halfField + oneAndHalfTile, 0, mmTargetHeight,
                90, 0, 180);
        identifyTarget(1, "Red Rear Wall",
                halfField + oneAndHalfTile, fullField, mmTargetHeight,
                90, 0, 0);
        identifyTarget(2, "Blue Audience Wall",
                halfField - oneAndHalfTile, 0, mmTargetHeight,
                90, 0, 180);
        identifyTarget(3, "Blue Rear Wall",
                halfField - oneAndHalfTile, fullField, mmTargetHeight,
                90, 0, 0);

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

    /*** (Vuforia-specific function)
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx
     * @param dy
     * @param dz
     * @param rx
     * @param ry
     * @param rz
     */
    @Override
    public void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    @Override
    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addLine("position")
            .addData("x","%.1f",x)
            .addData("y","%.1f",y);
        telemetry.addData("heading","%.1f", rotation);
        telemetry.addData("Brendan's heading","%.1f",smartAngleError(rotation, 0));
        telemetry.addData("Vuforia Target Visible", targetWasVisible);

    }
    public void measureVelocity() {
        double[] stateChange = imu.getStateChangeDegrees();
    }

    /**
     * Measure the state change from the IMU. Currently, this function only sets x and y velocities
     * based on the IMU. In practice, we should estimate these valoues. But the IMU implements its
     * own integration function.
     */
    public void measureIMUChange() {
        double[] stateChange = imu.getStateChangeDegrees();
        // Converts the robot's heading in its own frame to the field's frame.
        double fieldStateChange[] = robotToFieldFrame(stateChange[3], stateChange[4]);
        // x velocity in inches per second
        xVelocity = fieldStateChange[0];
        // y velocity in inches per second
        yVelocity = fieldStateChange[1];
        // Heading rate in degrees per second.
        rotationRate = stateChange[5];
    }

    public void measurePodChange() {
        double[] stateChange = odoPods.getStateChange();
        // Converts the robot's heading in its own frame to the field's frame.
        double fieldStateChange[] = robotToFieldFrame(stateChange[3], stateChange[4]);
        // x velocity in inches per second
        xVelocity = fieldStateChange[0];
        // y velocity in inches per second
        yVelocity = fieldStateChange[1];
        // Heading rate in degrees per second.
        rotationRate = stateChange[5];
    }

    /**
     * To understand this function, read about "Rotation Matrix" on Wikipedia and look at
     * the Desmos graph https://www.desmos.com/calculator/6evsqgs7qz
     * @param x the robot x value
     * @param y the robot y value
     * @return {fieldX, fieldY}
     *
     */
    public double[] robotToFieldFrame(double x,double y){
        double[] retVal ={
                 Math.cos(rotation *Math.PI/180)*x-Math.sin(rotation *Math.PI/180)*y,
                 Math.sin(rotation *Math.PI/180)*x+Math.cos(rotation *Math.PI/180)*y}; //how.TODO
        return retVal;
    }

    /**
     * To understand this function, read about "Rotation Matrix" on Wikipedia and look at
     * the Desmos graph https://www.desmos.com/calculator/6evsqgs7qz
     * @param x the robot x value
     * @param y the robot y value
     * @return {fieldX, fieldY}
     *
     */
    public double[] fieldToRobotFrame(double x,double y){
        double[] retVal ={
                Math.cos(rotation *Math.PI/180)*x+Math.sin(rotation *Math.PI/180)*y,
                -Math.sin(rotation *Math.PI/180)*x+Math.cos(rotation *Math.PI/180)*y}; //how.TODO
        return retVal;
    }
    /**
     * Updates the robot state estimate based on time elapsed.
     */
    private void updateState(){
        double thisTime = runtime.seconds();
        double deltaT = thisTime - stateTime;
        x += deltaT * xVelocity;
        y += deltaT * yVelocity;
        rotation += deltaT * rotationRate;
        stateTime = thisTime;
        // Over one second, we estimate that position uncertainty grows by 5 inch.
        positionUncertainty += deltaT*deltaT * 5;
        // Over one second, we estimate that heading uncertainty grows by 0.5 degrees.
        headingUncertainty += deltaT*deltaT * 5;
    }

    /**
     * Measure the Vuforia Heading
     */
    public void measureVuforiaHeading(){
        if(targetVisible) {
            double k = headingUncertainty / (headingUncertainty + vuforiaHeadingUncertainty); //Kalman gain for a Kalman filter
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            double vuforiaHeading = rotation.thirdAngle%360;
            double err = smartAngleError(vuforiaHeading, this.rotation);
            this.rotation += k*(err);
            headingUncertainty = (1-k)*headingUncertainty;

        }
    }

    /**
     * Computes an Angle Error calculation, accounting for wrap arounds.
     *
     * @param a angle a
     * @param b angle b
     * @return angle error (a-b, as close to zero as possible) Range: -180 to 180
     */
    public double smartAngleError(double a, double b){
        double diff = a - b;
        return diff - Math.round(diff/360.0) * 360.0;
    }

    /** Measures the robot position if possible
     *
     */
    public void measureVuforiaPosition(){
       if (targetVisible) {
           //https://kalmanfilter.net/kalman1d.html
           double k = positionUncertainty/(positionUncertainty+vuforiaPositionUncertainty); //another Kalman gain for a Kalman filter
           x += k*((lastLocation.getTranslation().get(0)/mmPerInch)-x);
           y += k*((lastLocation.getTranslation().get(1)/mmPerInch)-y);
           positionUncertainty = (1-k)*positionUncertainty;

       }
    }

    /**
     * Measures changes in state and incorporates them into our state estimates.
     */
    private void measureState(){
        measureVuforiaPosition();
        measureVuforiaHeading();
        measurePodChange();
    }

    @Override
    public void handleTracking() {
        // Always update and measure state.

        // Only
        // X and Y
        targetVisible = false;
        if ((runtime.seconds() - lastT) > loopSpeedHT) {
            // check all the trackable targets to see which one (if any) is visible.
            targetWasVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    targetWasVisible = true;
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    lastT=runtime.seconds();
                    break;
                }

            }
        }
        updateState();
        measureState();
        RobotLog.ii("Localizer", "State= %f %f %f %f %f %f %f %f", x, y, rotation, xVelocity, yVelocity, rotationRate,positionUncertainty, headingUncertainty);
        try {
            JSONObject state = new JSONObject()
                    .put("x", x)
                    .put("y", y)
                    .put("heading", rotation)
                    .put("xVelocity", xVelocity)
                    .put("yVelocity", yVelocity)
                    .put("headingRate", rotationRate)
                    .put("positionUncertainty", positionUncertainty)
                    .put("headingUncertainty", headingUncertainty)
                    .put("runTime", runtime.seconds())
                    .put("targetVisible", targetWasVisible)
                    .put("secondsSinceVuforia", runtime.seconds()-lastT);
            stateServer.addState(state);
        } catch (JSONException e) {
            RobotLog.ee("Localizer", "Error encoding json.");
        };
    }

    @Override
    public double getRotation() {

        return rotation;
    }

}

