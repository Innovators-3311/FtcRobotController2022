package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.enums.JunctionType;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;

class JunctionDistances {
    private static final double low = 7.9;
    // Touching the pole.
    private static final double close = 3.0;

    public static double getDistance(JunctionType junctionType){
        if (junctionType == JunctionType.HIGH) return close;
        if (junctionType == JunctionType.MEDIUM) return close;
        if (junctionType == JunctionType.LOW) return low;
        return 0;
    }
}

public class JunctionHomingController {

    public enum CenteringState
    {
        IDLE,
        TURNING,
        FIRST_EDGE,
        RECENTER,
        DONE,
    }

    public enum AligningState{
        IDLE,
        CENTERING,
        DRIVING, FAILED, DONE
    }

    public CenteringState centeringState = CenteringState.IDLE;
    public AligningState aligningState = AligningState.IDLE;

    private JunctionType junctionType;

    private static final int SWEEP_ANGLE = 90;
    private static final double POSITION_TOLERANCE = 1.0;
    private static final double MAX_DISTANCE = 24;

    private final Telemetry telemetry;
    private final MecanumDriveBase mecanumDriveBase;
    private BNO055IMU imu;
    private final DistanceSensor distanceSensorCenter;
    private final RelativeDriveController relativeDrive;
    private JunctionType junction = JunctionType.LOW;

    private double powerRate = 0;
    private double power = 0.6;
    private double degrees;
    private double firstPoleEdge =0.0;
    private double secondPoleEdge =0.0;
    private double centerAngle = 0.0;
    private double minDistance = 1e9;

    Orientation lastAngles = new Orientation();
    double  globalAngle;


    public JunctionHomingController(Telemetry telemetry, MecanumDriveBase mecanumDriveBase, HardwareMap hardwareMap, RelativeDriveController relativeDriveController) {
        this.telemetry = telemetry;
        this.mecanumDriveBase = mecanumDriveBase;
        distanceSensorCenter = hardwareMap.get(DistanceSensor.class, "distanceSensorCenter");
        this.relativeDrive = relativeDriveController;
        initImu(hardwareMap);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Init the IMU code
     */
    private void initImu(HardwareMap hardwareMap)
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     * Call the state if we're aligning to a pole.
     */
    public void delegateAligningState(){
        if (aligningState == AligningState.CENTERING) stateCentering();
        if (aligningState == AligningState.DRIVING) stateDriving();
    }

    /**
     * Handle the centering state.
     */
    public void stateCentering(){
        delegateCenteringState();

        if (centeringState == CenteringState.DONE){
            // Transition to Driving.  Set up Relative Drive.
            aligningState = AligningState.DRIVING;
            RobotLog.ii("JunctionHomingController", "Centered Complete. Entering Driving.");
            double measDist = getDistance();

            if (measDist > MAX_DISTANCE){
                RobotLog.ee("JunctionHomingController",
                        "After junction alignment, measured Distance (%f) > max distance (%f)",
                        measDist, MAX_DISTANCE);
                aligningState = AligningState.FAILED;
            }

            double forward = measDist - JunctionDistances.getDistance(junctionType);

            relativeDrive.setRelativeTarget(-forward, 0, 0);
            relativeDrive.speedFactor = power;
        }
    }

    private void stateDriving(){
        relativeDrive.handleRelativeDrive();
        if (!relativeDrive.driving()) {
            mecanumDriveBase.driveMotors(0,0,0,0);
            RobotLog.ii("JunctionHomingController", "Aligned to pole.");
            aligningState = AligningState.DONE;
        }
    }

    public void setJunctionType(JunctionType junctionType){
        this.junctionType = junctionType;
    }

    public boolean alignToPole(JunctionType junctionType, double sweepAngle){
        // BLOCKING FUNCTION CALL
        // Turn left and look for a junction.
        double measDist = junctionCenter(sweepAngle, power);

        // Compute distance to the pole.
        double forward = measDist - JunctionDistances.getDistance(junctionType);

        RobotLog.ii("JunctionHomingController", "Approaching %s junction. Measured distance: %f",
                junctionType.toString(), measDist);

        relativeDrive.setRelativeTarget(-forward, 0, 0);
        relativeDrive.speedFactor = power;
        // BLOCKING FUNCTION CALL
        while (relativeDrive.driving()) {
            relativeDrive.handleRelativeDrive();
        }

        mecanumDriveBase.driveMotors(0, 0, 0, 0);
        RobotLog.ii("JunctionHomingController", "Completed drive. Measured distance: %f",
                getDistance());

        return (measDist < MAX_DISTANCE);
    }

    /**
     * Align to the pole on the left corner.
     *
     * @param junctionType The type of junction we're aligning to (LOW, MEDIUM, HIGH)
     */
    public boolean alignToPoleLeft(JunctionType junctionType) {
        return alignToPole(junctionType, SWEEP_ANGLE);
    }

    /**
     * Align to the pole on the left corner.
     *
     * @param junctionType The type of junction we're aligning to (LOW, MEDIUM, HIGH)
     */
    public boolean alignToPoleRight(JunctionType junctionType) {
        return alignToPole(junctionType, -SWEEP_ANGLE);
    }

    /**
     * Check to see if the sensor detects an object within the range provided.
     * @param innerBound minimum range detection.
     * @param outerBound maximum range of detection.
     */
    private double checkDistance(double innerBound, double outerBound)
    {
        double distance = getDistance();

        if (distance < outerBound && distance > innerBound)
        {
            distance = distanceSensorCenter.getDistance(DistanceUnit.INCH);
        }
        else
        {
            distance = -1;
        }

        return distance;
    }

    /**
     * Gets the distance in inches.
     *
     * @return double the measured distance in inches
     */
    public double getDistance(){
        return distanceSensorCenter.getDistance(DistanceUnit.INCH);
    }

    /**
     * Nonblocking idle function
     */
    public void stateIdle(){
        // Idling . . . Do nothing?
    }

    public void setDegrees(double degrees) {
        if (degrees < 0)
        {   // turn right.
            powerRate = power;
        }
        else if (degrees > 0)
        {   // turn left.
            powerRate = -power;
        }
        this.degrees = degrees;

        RobotLog.ii("JunctionHomingController", "Set degrees to %f", this.degrees);
    }

    /**
     * Check if the turn is complete.
     *
     * @return is it complete or still working?
     */
    public boolean checkTurnComplete(){
        if (Math.abs(getAngle()) > Math.abs(degrees)) {
            // STOP
            mecanumDriveBase.driveMotors(0, 0, 0, 0);
            centeringState = CenteringState.DONE;
            return true;
        }
        return false;
    }

    /**
     * Nonblocking handler for the turning state.
     */
    public void stateTurning(){
        // set power to rotate.
        mecanumDriveBase.driveMotors(0, powerRate, 0, 1);
        if (checkTurnComplete()){
            return;
        }

        double distance = checkDistance(0, MAX_DISTANCE);
        if (distance != -1){
            firstPoleEdge = getAngle();
            RobotLog.ii("JunctionHomingController", "Found First Edge. Angle: %f", firstPoleEdge);
            centeringState = CenteringState.FIRST_EDGE;
        }
    }

    /**
     * First Edge state control.
     */
    public void stateFirstEdge(){
        // set power to rotate.
        mecanumDriveBase.driveMotors(0, powerRate, 0, 1);
        if (checkTurnComplete()){
            return;
        }

        double distance = checkDistance(0, MAX_DISTANCE);
        minDistance = Math.min(minDistance, getDistance());

        if (distance == -1){
            secondPoleEdge = getAngle();
            RobotLog.ii("JunctionHomingController", "Found Second Edge. Angle: %f", secondPoleEdge);
            centerAngle = (firstPoleEdge + secondPoleEdge) / 2.0;
            centeringState = CenteringState.RECENTER;
        }
    }

    /**
     * Recenter state control.
     */
    public void stateRecenter(){
        // set power to rotate.
        mecanumDriveBase.driveMotors(0, -powerRate, 0, 1);

        if (Math.abs(getAngle()) < Math.abs(centerAngle)) {
            mecanumDriveBase.driveMotors(0, 0, 0, 0);
            centeringState = CenteringState.DONE;
        }
    }

    /**
     * Called when we want to be in a non-blocking for loop.
     */
    public void delegateCenteringState(){
        if(centeringState == CenteringState.IDLE) stateIdle();
        if(centeringState == CenteringState.TURNING) stateTurning();
        if(centeringState == CenteringState.FIRST_EDGE) stateFirstEdge();
        if(centeringState == CenteringState.RECENTER) stateRecenter();
    }

    public void startCentering() {
        // Start turning!
        centeringState = CenteringState.TURNING;
        minDistance = 1e9;

        // restart imu movement tracking.
        resetAngle();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     * @param power Power setting from 0 to 1
     *
     * @return double Distance to the pole. -1 if no pole.
     */
    public double junctionCenter(double degrees, double power)
    {
        setDegrees(degrees);
        setPower(power);
        startCentering();

        while(centeringState != CenteringState.DONE)
            delegateCenteringState();

        return minDistance;
    }

    private void setPower(double power)
    {
        this.power = power;
    }

    /**
     * Copied in from the OpMode class.
     *
     * @param milliseconds how long to sleep
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }

    }

    /**
     * Handle the logic for the gamepad.
     *
     * @param driver The driver's gamepad
     * @param accessory The accessory gamepad
     */
    public void handleGamepad(Gamepad driver, Gamepad accessory){
        if      (accessory.y){setJunctionType(JunctionType.HIGH);}
        else if (accessory.x){setJunctionType(JunctionType.MEDIUM); }
        else if (accessory.b){setJunctionType(JunctionType.LOW);}

        if(driver.left_bumper){
            if (aligningState == AligningState.IDLE) {
                RobotLog.ii("JunctionHomingController", "Beginning Junction Alignment from IDLE");
                aligningState = AligningState.CENTERING;
                setDegrees(90);
                startCentering();
            }
            // Calls the handling function for the appropriate state.
            delegateAligningState();
        } else if (driver.right_bumper){
            if (aligningState == AligningState.IDLE) {
                RobotLog.ii("JunctionHomingController", "Beginning Junction Alignment from IDLE");
                aligningState = AligningState.CENTERING;
                setDegrees(-90);
                startCentering();
            }
            // Calls the handling function for the appropriate state.
            delegateAligningState();
        }
        else {
            aligningState = AligningState.IDLE;
            centeringState = CenteringState.IDLE;
        }
    }

}
