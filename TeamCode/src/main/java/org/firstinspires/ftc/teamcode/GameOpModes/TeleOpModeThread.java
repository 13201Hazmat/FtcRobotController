package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadDriveTrainController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.ParkingArm;
import org.firstinspires.ftc.teamcode.SubSystems.VisionSensor;
import org.firstinspires.ftc.teamcode.TestOpModes.VisionAprilTag;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "Hazmat TeleOp Thread", group = "00-Teleop")
public class TeleOpModeThread extends LinearOpMode {

    public GamepadController gamepadController;
    public GamepadDriveTrainController gamepadDriveTrainController;
    public DriveTrain driveTrain;
    public Launcher launcher;
    public ParkingArm parkingArm;
    public VisionSensor visionSensor;
    public VisionAprilTag visionAprilTagBack;
    public Lights lights;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        gamepadDriveTrainController.start();

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            //outtakeArm.backPlateAlignDown();

            while (opModeIsActive()) {
                gamepadController.runByGamepadControl();
                //lights.setPattern(Lights.REV_BLINKIN_PATTERN.D);


                /*if (gameTimer.time() > 85000 && gameTimer.time() < 90000) {
                    //lights.setPattern(Lights.REV_BLINKIN_PATTERN.END_GAME);
                }*/

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
                }
            }
        }
        gamepadDriveTrainController.exit();
        GameField.poseSetInAutonomous = false;
    }

    public void initSubsystems(){

        telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        telemetry.addLine("Intake Initialized");
        telemetry.update();

        telemetry.addLine("Magazine Initialized");
        telemetry.update();

        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        telemetry.addLine("Climber Initialized");
        telemetry.update();

        launcher = new Launcher(hardwareMap, telemetry);
        telemetry.addLine("Launcher Initialized");
        telemetry.update();

        parkingArm = new ParkingArm(hardwareMap, telemetry);
        telemetry.addLine("ParkingArm Initialized");
        telemetry.update();

        /* Create VisionAprilTag */
        //visionAprilTagBack = new VisionAprilTag(hardwareMap, telemetry, "Webcam 2");
        //telemetry.addLine("Vision April Tag Front Initialized");
        //telemetry.update();

        /* Create VisionSensor */
        visionSensor = new VisionSensor(hardwareMap, telemetry);
        telemetry.addLine("Vision Sensor Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadDriveTrainController = new GamepadDriveTrainController(gamepad1, driveTrain, this);
        telemetry.addLine("Gamepad DriveTrain Initialized");
        telemetry.update();

        gamepadController = new GamepadController(gamepad1, gamepad2, launcher, visionSensor, lights, telemetry, this);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            driveTrain.pose = GameField.currentPose;
            //driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.pose = startPose;
            //driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat TeleOpMode");
            telemetry.addData("Game Timer : ", gameTimer.time());

            driveTrain.printDebugMessages();
            launcher.printDebugMessages();
            parkingArm.printDebugMessages();
            visionSensor.printDebugMessages();
            //visionAprilTagBack.printdebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }

}
