package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;

public class JunctionHomingController {
    private Telemetry telemetry;
    private MecanumDriveBase mecanumDriveBase;
    private BNO055IMU imu;
    private DistanceSensor distanceSensorCenter;

    Orientation lastAngles = new Orientation();
    double  globalAngle;


    public JunctionHomingController(Telemetry telemetry, MecanumDriveBase mecanumDriveBase, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.mecanumDriveBase = mecanumDriveBase;
        distanceSensorCenter = hardwareMap.get(DistanceSensor.class, "distanceSensorCenter");
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
     * Check to see if the sensor detects an object within the range provided.
     * @param innerBound minimum range detection.
     * @param outerBound maximum range of detection.
     */
    private double checkDistance(double innerBound, double outerBound)
    {
        double distance = distanceSensorCenter.getDistance(DistanceUnit.INCH);

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
         * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
         * @param degrees Degrees to turn, + is left - is right
         * @param power Power setting from 0 to 1
         * @param sensor Boolean indicating if to stop on center sensor detection
         */
        private void basicRotate(double degrees, double power, boolean sensor)
        {
            double powerRate = 0;
            double distance = 0;
            double firstPole = 0;
            double secondPole  = 0;
            boolean recenterOnPole = false;
            boolean foundPole = false;

            // restart imu movement tracking.
            resetAngle();

            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            if (degrees < 0)
            {   // turn right.
                telemetry.addData("Turn right", "");
                telemetry.update();
                powerRate = power;
            }
            else if (degrees > 0)
            {   // turn left.
                telemetry.addData("Turn left", "");
                telemetry.update();
                powerRate = -power;
            }
            else return; //angle is 0

            // set power to rotate.
            mecanumDriveBase.driveMotors(0, powerRate, 0, 1);


            // rotate until turn is completed.
            if (!sensor)
            {
                telemetry.addData("Simple turn", "");
                telemetry.update();
                // On right turn we have to get off zero first.
                while (getAngle() == 0) {}

                while (true)
                {
                    if (Math.abs(getAngle()) > Math.abs(degrees))
                    {
                        mecanumDriveBase.driveMotors(0, 0, 0, 0);
                        telemetry.addData("Stop", "");
                        telemetry.update();
                        break;
                    }
                }
            }
            else           //Following code only used if sensor is in play
            {
                // left turn.
                while (Math.abs(getAngle()) < Math.abs(degrees))
                {
                    if (sensor)
                    {
                        telemetry.addData("Sensor in use","");
                        telemetry.update();
                        distance = checkDistance(0, 24);
                        if ((distance != -1) && !foundPole)
                        {
                            //telemetry.addData("found pole", "");
                            //telemetry.update();
                            firstPole = getAngle();
                            telemetry.addData("Angle #1:", firstPole);
                            foundPole = true;
                        }

                        distance = checkDistance(0, 24);
                        if (foundPole && distance == -1)
                        {
                            //telemetry.addData("lost pole", "");
                            //telemetry.update();
                            mecanumDriveBase.driveMotors(0, 0, 0, 0);
//                            sleep(100);TODO
                            secondPole = getAngle();
                            telemetry.addData("Angle #2:", secondPole);
                            recenterOnPole = true;
                            break;
                        }
                    }
                }
            }

            // turn the motors off.
            mecanumDriveBase.driveMotors(0, 0, 0, 0);

            // wait for rotation to stop.
//            sleep(300);TODO

            // reset angle tracking on new heading.
            resetAngle();

            //turn back in the reverse direction to center on the scanned pole.
            if (recenterOnPole && sensor)
            {
                //telemetry.addData("recentering on pole", "");
                //telemetry.update();

                if (degrees > 0)
                {
                    double angleCorrection = (firstPole - secondPole) / 2;
                    basicRotate(angleCorrection, 0.2, false);
                    int temp = (int)angleCorrection;
                    telemetry.addData("Angle Correction:", angleCorrection + " int: " + temp);
                }
                else
                {
                    double angleCorrection = (firstPole - secondPole) / 2;
                    basicRotate(angleCorrection, 0.2, false);

                    int temp = (int)angleCorrection;
                    telemetry.addData("Angle Correction:", angleCorrection + " int: " + temp);
                }
                telemetry.update();
            }
        }

    }
