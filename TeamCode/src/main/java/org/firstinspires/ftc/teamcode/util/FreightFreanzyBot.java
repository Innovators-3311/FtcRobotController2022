package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.controllers.PacManTurnToPos;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FreightFreazyBot", group="!")
public class FreightFreanzyBot extends OpMode

{
    private MecanumDriveBase mecanumDriveBase = null;
    private DcMotor arm;
    private DcMotor wheel;

    public void init()
    {
        telemetry.addData("Status", "Initialized");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap, false);
    }
    @Override
    public void loop()
    {
        mecanumDriveBase.gamepadController(gamepad1);
        driveArm(gamepad1);
        spinnyWheel(gamepad1);
    }

    private void driveArm(Gamepad gamepad)
    {
        if (gamepad.right_trigger != 0)
        {
            arm.setPower(gamepad.right_trigger);
        }
        else if (gamepad.left_trigger != 0)
        {
            arm.setPower(-gamepad.left_trigger);
        }
        else
        {
            arm.setPower(0);
        }
    }

    private void spinnyWheel(Gamepad gamepad)
    {
        if (gamepad.right_bumper)
        {
            wheel.setPower(1);
        }
        else if (gamepad.left_bumper)
        {
            wheel.setPower(-1);
        }
        else
        {
            wheel.setPower(0);
        }
    }
}