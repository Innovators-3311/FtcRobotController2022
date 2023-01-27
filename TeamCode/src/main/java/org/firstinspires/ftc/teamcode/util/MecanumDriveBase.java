package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.localizers.CombinedLocalizer;

public class MecanumDriveBase {
    private static final DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    private CombinedLocalizer localizer;
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor rf;
    public double leftPowerFront  = 0;
    public double rightPowerFront = 0;
    public double rightPowerBack  = 0;
    public double leftPowerBack   = 0;//TODO: Figure out how to not use Localizer here //sam like a flag
    public double speedFactor     = 0;

    public MecanumDriveBase(HardwareMap hardwareMap, boolean autonomous)
    {
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run Without Encoders

        setMotorMode(this.runMode);

        // Brake when power set to Zero
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * We tend to set all the motor modes at once, so break it out using "extract Method" under the
     * refactor menu
     *
     * @param runMode The runMode to set all motors to.
     */
    private void setMotorMode(DcMotor.RunMode runMode) {
        lf.setMode(runMode);
        rf.setMode(runMode);
        lb.setMode(runMode);
        rb.setMode(runMode);
    }


    /**
     * Standard controls from a gamepad
     *
     * @param gamepad - the gamepad you want to control the drive base
     */
    public void gamepadController(Gamepad gamepad) {

          double drive = -gamepad.left_stick_y;
          double turn = gamepad.right_stick_x;
          double strafe = gamepad.left_stick_x;
          speedFactor = 1 - (.5*gamepad.right_trigger);
          driveMotors(drive, turn, strafe, speedFactor);
      }

    /**
     * Drive the motors according to drive, turn, strafe inputs.
     *
     * @param drive forward / backward (-1 to 1)
     * @param turn how much to turn left or right (heading) (-1 to 1)
     * @param strafe strafe (left or right = -1 to 1)
     * @param speedFactor scale factor that is applied to all motor powers (0 to 1)
     */
      public void driveMotors(double drive,double turn,double strafe,double speedFactor)
      {

          // Negative turn is left
          leftPowerFront  = (drive + turn + strafe) * speedFactor;
          rightPowerFront = (drive - turn - strafe) * speedFactor;
          leftPowerBack   = (drive + turn - strafe) * speedFactor;
          rightPowerBack  = (drive - turn + strafe) * speedFactor;


          double maxAbsVal = maxAbsVal(leftPowerFront, leftPowerBack,
                                       rightPowerFront, rightPowerBack);

          // MaxAbsVal should only REDUCE power sent to the wheels, not increase it.
          maxAbsVal = Math.max(1.0, maxAbsVal);

          lf.setPower(leftPowerFront/maxAbsVal);
          rf.setPower(rightPowerFront/maxAbsVal);
          lb.setPower(leftPowerBack/maxAbsVal);
          rb.setPower(rightPowerBack/maxAbsVal);
      }

    /**
     * maxAbsVal returns the maximum absolute value among an arbitrary number of arguments.
     *
     * @param values an arbitrary number of values.
     * @return the maximum absolute value among the numbers.
     */
      public static double maxAbsVal(double ... values){
          double mav = Double.NEGATIVE_INFINITY;
          for (double val: values) {
              mav = Math.max(mav, Math.abs(val));
          }
          return mav;
      }

    /**
     * report drivebase telemetry
     *
     * @param telemetry the telemetry object we're reporting to.
     */
      public void driveBaseTelemetry(Telemetry telemetry)
      {
        telemetry.addData("Motors", "lf(%.2f), rf(%.2f), lb(%.2f), rb(%.2f)", leftPowerFront, rightPowerFront, leftPowerBack, rightPowerBack);
        telemetry.addData("Speed control", speedFactor);
      }
}

