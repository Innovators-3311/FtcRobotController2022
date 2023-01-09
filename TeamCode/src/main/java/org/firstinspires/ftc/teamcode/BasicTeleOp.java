package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.DriveForTest;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BackUpCode", group="!")
//@Disabled
public class BasicTeleOp extends OpMode

{
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private DriveForTest driveForTest;

    public void init() {
        // Commented out till merge with master
        telemetry.addData("Status", "Initialized");



//        localizer = new IntegratedLocalizerIMU(hardwareMap);
//        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        towerController = new TowerController(hardwareMap, telemetry);
//        localizer = new LocalizerIMU(hardwareMap);
//        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
//        double max;
        driveForTest = new DriveForTest(hardwareMap);

//        Logging.setup();
//        Logging.log("Starting Drive Circle Logging");

//        Thread uBarThread = new UBarThread(hardwareMap , telemetry, gamepad2);

//        uBarThread.start();

    }
    @Override
    public void loop() {
//        localizer.displayTelemetry(telemetry);
//        localizer.handleTracking();
//        mecanumDriveBase.gamepadController(gamepad1);
//        mecanumDriveBase.driveBaseTelemetry(telemetry);
        towerController.handleGamepad(gamepad2, telemetry);
//        telemetry.addData("BasicTeleOp heading", localizer.getHeading() );
//        pacMan.handlePacMan(gamepad1, telemetry);
        driveForTest.drive(gamepad1);
        telemetry.update();
    }
}
