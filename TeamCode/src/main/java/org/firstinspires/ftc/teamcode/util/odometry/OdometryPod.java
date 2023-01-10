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
     * Odometry Pod Constructor
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

    public double getDistanceChangeInches() {
        double distanceChange = inchesPerTick * (pod.getCurrentPosition() - lastPos);
        lastPos = pod.getCurrentPosition();
        return distanceChange;
    }

    /**
     * Odometry Pod Constructor
     *
     * @param hardwareMap The hardwareMap
     * @param podName The name of the pod to initialize
     */
    public OdometryPod(HardwareMap hardwareMap, String podName)
    {
        this(hardwareMap, podName, false);
    }
}
