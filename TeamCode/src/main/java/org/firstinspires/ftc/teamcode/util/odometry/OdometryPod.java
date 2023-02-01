package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OdometryPod {
    public static final double ticksPerRev = 8192;
    public static final double circumference = 6.283;
    public static final double inchesPerTick = circumference/ticksPerRev;
    public DcMotor pod;
    private int lastPos;
    private double encoderSign = 1;


    /**
     * Odometry Pod Constructor with encoderSignInverted option.
     *
     * @param hardwareMap The hardwareMap
     * @param podName The name of the pod motor
     * @param encoderSignInverted Whether the pod is inverted
     */
    public OdometryPod(HardwareMap hardwareMap, String podName, boolean encoderSignInverted) {
        pod = hardwareMap.get(DcMotor.class, podName);
        lastPos = pod.getCurrentPosition();
        if (encoderSignInverted)
        {
            encoderSign = -1;
        }
    }

    /**
     * Odometry Pod Constructor
     *
     * This constructor assumes the encoder sign is not inverted.
     *
     * @param hardwareMap The hardwareMap
     * @param podName The name of the pod to initialize
     */
    public OdometryPod(HardwareMap hardwareMap, String podName)
    {
        this(hardwareMap, podName, false);
    }

    /**
     * Call this to reset the pod's encoder to 0. Note: Stops the associated motor!
     */
    public void resetEncoder() {
        pod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastPos = 0;
    }

    /**
     * Returns the total distance this pod has traveled since it was last reset to zero.
     * 
     * @return inches traveled
     */
    public double getDistanceInches() {
        return pod.getCurrentPosition() * inchesPerTick * encoderSign;
    }

    /**
     * Get the distance this odometryt pod has moved since the last reading.
     * 
     * @return distance in inches.
     */
    public double getDistanceChangeInches() {
       /* Watch out! getCurrentPosition returns an integer, which has a maximum value of 2147483647.
        We don't know what happens if we exceed that number.  Likely that we restart at -2147483648!
        If it's 8192 ticks per rotation and \pi*2 inches per rotation, that works out to a rollover 
        of 52 miles.

        **Good news though.** We'd have to drive 26 miles in a match to reach that number.

        */
        double distanceChange = inchesPerTick * (pod.getCurrentPosition() - lastPos);
        lastPos = pod.getCurrentPosition();
        return distanceChange;
    }


}
