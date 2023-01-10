package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PositionChangeSensor;

public class OdometryPodsSensor implements PositionChangeSensor {
    /**
     * IMPORTANT:
     * This class assumes that there are three odometry pods on the front, left, and right side of the robot.
     **/
    private OdometryPod leftPod;
    private OdometryPod rightPod;
    private OdometryPod centerPod;
    private ElapsedTime runtime = new ElapsedTime();
    private double last_measurement;

    // Distance between wheels
    private static final double width = 14;
    // Distance from the center of the robot to the front wheel
    private static final double frontPodDistance = 2;




    public OdometryPodsSensor(HardwareMap hardwareMap) {
        leftPod  = new OdometryPod(hardwareMap, "lb");
        rightPod = new OdometryPod(hardwareMap, "rf");
        centerPod = new OdometryPod(hardwareMap, "lf");
        last_measurement = runtime.seconds();
    }


    /** Gets the Robot's Estimated State Change.
     *
     * State change is returned in terms of the Robot's motion in its forward direction, strafe
     * (left is positive), and heading change (in radians)
     *
     * @return double[] {forward, strafe, rotation, forwardRate, strafeRate, rotationRate}
     */
    public double[] getStateChange() {
        double distanceChangeLeft  = leftPod.getDistanceChangeInches();
        double distanceChangeRight = rightPod.getDistanceChangeInches();
        double deltaT = runtime.seconds() - last_measurement;
        last_measurement = runtime.seconds();
        double forward = (distanceChangeLeft + distanceChangeRight)/2;
        double rotationChange = -(distanceChangeLeft - distanceChangeRight) / width; //left - right because we want a positive change to the heading(turning right).
        double strafe = centerPod.getDistanceChangeInches() - (rotationChange*frontPodDistance);//?
        double forwardRate = forward /deltaT;
        double strafeRate = strafe / deltaT;
        double rotationRate = rotationChange / deltaT;
        double[] retVal = {forward,strafe,rotationChange,forwardRate, strafeRate, rotationRate};
        return retVal;
    }
    /** Gets the Robot's Estimated State Change.
     *
     * State change is returned in terms of the Robot's motion in its forward direction, strafe
     * (left is positive), and heading change (in degrees).
     *
     * @return double[] {forward, strafe, heading}
     */
    public double[] getStateChangeDegrees() {
        double[] retVal = getStateChange();
        retVal[2] *= 180.0 / Math.PI;
        return retVal;
        // converts radians into degrees to be more compatible with our code.
    }

}
