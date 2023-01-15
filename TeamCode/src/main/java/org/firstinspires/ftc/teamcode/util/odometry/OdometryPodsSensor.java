package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.PositionChangeSensor;
import org.firstinspires.ftc.teamcode.util.localizers.StateServer;
import org.json.JSONException;
import org.json.JSONObject;

public class OdometryPodsSensor implements PositionChangeSensor {
    /**
     * IMPORTANT:
     * This class assumes that there are three odometry pods on the front, left, and right side of the robot.
     **/
    private OdometryPod leftPod;
    private OdometryPod rightPod;
    private OdometryPod centerPod;
    private StateServer stateServer;
    private ElapsedTime runtime = new ElapsedTime();
    private double last_measurement;

    // Minimum time between state logging (otherwise it's stupid-fast).
    private final double MIN_STATE_PERIOD = 0.1;
    private double lastStateLog = 0.0;

    // Distance between wheels
    private static final double width = 14;
    // Distance from the center of the robot to the front wheel
    private static final double frontPodDistance = 2;




    public OdometryPodsSensor(HardwareMap hardwareMap) {
        // change lf rb
        leftPod   = new OdometryPod(hardwareMap, "lb", true);
        rightPod  = new OdometryPod(hardwareMap, "lf");
        centerPod = new OdometryPod(hardwareMap, "rf");
        stateServer = StateServer.getInstance();
        last_measurement = runtime.seconds();
    }


    public void logState(){
        if(stateServer.valid && (runtime.seconds()-lastStateLog) > MIN_STATE_PERIOD)
        {
            try
            {
                JSONObject state = new JSONObject().put("runTime", runtime.seconds())
                        .put("leftPodValue", leftPod.getLastPos())
                        .put("rightPodValue", rightPod.getLastPos())
                        .put("centerPodValue", centerPod.getLastPos())
                        .put("type", "OdometryPodsSensor");
                stateServer.addState(state);
                lastStateLog = runtime.seconds();
            } catch (JSONException e)
            {
                RobotLog.ee("Localizer", "Error encoding json.");
            }
        }

    }

    /** Gets the Robot's Estimated State Change.
     *
     * State change is returned in terms of the Robot's motion in its forward direction, strafe
     * (left is positive), and heading change (in radians)
     *
     * @return double[] {forward, strafe, heading, forwardRate, strafeRate, headingRate}
     */
    public double[] getStateChange() {
        double distanceChangeLeft  = leftPod.getDistanceChangeInches();
        double distanceChangeRight = rightPod.getDistanceChangeInches();
        double deltaT = runtime.seconds() - last_measurement;
        last_measurement = runtime.seconds();
        double forward = (distanceChangeLeft + distanceChangeRight)/2;
        double headingChange = (distanceChangeLeft - distanceChangeRight) / width; //left - right because we want a positive change to the heading(turning right).
        double strafe = centerPod.getDistanceChangeInches() - (headingChange*frontPodDistance);//?
        double forwardRate = forward /deltaT;
        double strafeRate = strafe / deltaT;
        double headingRate = headingChange/deltaT;
        double[] retVal = {forward,strafe,headingChange,forwardRate, strafeRate, headingRate};
        this.logState();
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
