package org.firstinspires.ftc.teamcode.util.odometry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PositionChangeSensor;

public class OdometryPodsSensor implements PositionChangeSensor {
    /**
     * IMPORTANT:
     * This class assumes that there are three odometry pods on the front, left, and right side of the robot.
     **/
    private final OdometryPod leftPod;
    private final OdometryPod rightPod;
    private final OdometryPod centerPod;
    private final ElapsedTime runtime = new ElapsedTime();
    private double last_measurement;

    // Distance between wheels
    private static final double width = 14;
    // Distance from the center of the robot to the front wheel
    private static final double frontPodDistance = 2;


    public OdometryPodsSensor(HardwareMap hardwareMap) {
        // change lf rb
        leftPod   = new OdometryPod(hardwareMap, "lb", true);
        rightPod  = new OdometryPod(hardwareMap, "lf");
        centerPod = new OdometryPod(hardwareMap, "rf");
        last_measurement = runtime.seconds();
    }

    /** Gets the Robot's Total Odometry State.
     *
     * This function returns the total odometry state.  This can't be used to know where the robot
     * is because the specific order turns and forward / backwards / strafe makes a big difference
     * in terms of the final position. But it can be used to implement a simple drive operation.
     *
     * @return double[] {forward, strafe, heading, forwardRate, strafeRate, headingRate}
     */
    public double[] getState() {
        double distanceLeft  = leftPod.getDistanceInches();
        double distanceRight = rightPod.getDistanceInches();
        double distanceFront = centerPod.getDistanceInches();
        double deltaT = runtime.seconds();
        return computeStateChange(distanceLeft, distanceRight, distanceFront, deltaT);
    }


    /** Gets the Robot's Estimated State Change.
     *
     * State change is returned in terms of the Robot's motion in its forward direction, strafe
     * (left is positive), and heading change (in radians). This function returns the change
     * in each of these values since last time the function was called.
     *
     * @return double[] {forward, strafe, heading, forwardRate, strafeRate, headingRate}
     */
    public double[] getStateChange() {
        double distanceChangeLeft  = leftPod.getDistanceChangeInches();
        double distanceChangeRight = rightPod.getDistanceChangeInches();
        double distanceChangeFront = centerPod.getDistanceChangeInches();
        double deltaT = runtime.seconds() - last_measurement;
        return computeStateChange(distanceChangeLeft, distanceChangeRight, distanceChangeFront, deltaT);
    }


    @NonNull
    private double[] computeStateChange(double distanceLeft, double distanceRight, double distanceFront, double deltaT) {
        last_measurement = runtime.seconds();
        double forward = (distanceLeft + distanceRight)/2;
        double headingChange = (distanceLeft - distanceRight) / width;
        //left - right because we want a positive change to the heading(turning right).
        double strafe = distanceFront - (headingChange*frontPodDistance);//?
        double forwardRate = forward / deltaT;
        double strafeRate = strafe / deltaT;
        double headingRate = headingChange / deltaT;

        return new double[]{forward, strafe, headingChange, forwardRate, strafeRate, headingRate};
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
