package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.GamepadDriveTrainController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.VisionLimeLight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@TeleOp (name = "Test Vision Limelight", group = "Testing")
public class TestVisionLimeLight extends LinearOpMode {
    public VisionLimeLight visionLimeLight;
    public VisionPortal visionPortal;
    public TestGamepadController gamepadController;
    public GamepadDriveTrainController gamepadDriveTrainController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;

    ColorRange selectedTargetColor = ColorRange.YELLOW;

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;

        initSubsystems();
        visionLimeLight.startLimelight();
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        gamepadDriveTrainController.start();

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            visionLimeLight.locateNearestSampleFromRobot();

            if (!gamepadController.gp1GetStart()) {
                if (gamepadController.gp1GetRightBumperPress()) {
                    switch (intakeArm.intakeArmState) {
                        case INIT:
                        case EJECT_OR_PRE_TRANSFER:
                        case LOWEST:
                        case DYNAMIC:
                        case POST_TRANSFER:
                        case TRANSFER:
                            //GameField.turboFactor = false;
                            intakeSlides.moveIntakeSlidesToRange(visionLimeLight.calculateYExtensionFactorFromLookUp(visionLimeLight.yPos));
                            intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                            intakeArm.moveSwivelTo(visionLimeLight.angle);
                            break;
                        case PRE_PICKUP:
                            //GameField.turboFactor = true;
                            //if (intakeArm.intakeGripAutoClose) {
                            if (intakeArm.intakeGripState == IntakeArm.GRIP_STATE.OPEN) {
                                //PICKUP SEQUENCE
                                intakeArm.moveArm(IntakeArm.ARM_STATE.PICKUP);
                                safeWaitMilliSeconds(200);
                                intakeArm.closeGrip();
                                safeWaitMilliSeconds(100);
                                intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                            } else {
                                intakeArm.openGrip();
                            }
                            break;
                        case PICKUP:
                            intakeArm.toggleGrip();
                            if (intakeArm.intakeGripState == IntakeArm.GRIP_STATE.OPEN) {
                                intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                            }
                            break;
                    }
                }
            }

            if (gamepadController.gp1GetLeftBumperPress()){
                //intakeOuttakeController.closeGripAndMoveIntakeArmToPreTransfer();
                intakeArm.closeGrip();
                intakeArm.moveArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);

                //intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakePreDrop();
                intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
                safeWaitMilliSeconds(200+ 100* intakeSlides.slideExtensionFactor());
                intakeArm.moveArm(IntakeArm.ARM_STATE.TRANSFER);
                safeWaitMilliSeconds(800);
                intakeArm.openGrip();
                safeWaitMilliSeconds(500);
                intakeArm.moveArm(IntakeArm.ARM_STATE.POST_TRANSFER);
            }

            if(gamepadController.gp1GetDpad_upPress()){
                outtake.extendVisionArm();
            }

            if(gamepadController.gp1GetDpad_downPress()){
                outtake.retractVisionArm();
            }

            intakeSlides.INTAKE_SLIDE_DELTA = 0.01;
            if(gamepadController.gp1GetTrianglePress()){
                intakeSlides.moveIntakeSlidesForward();
            }
            if(gamepadController.gp1GetCrossPress()){
                intakeSlides.moveIntakeSlidesBackward();
            }

            printDebugMessages();
        }
        visionLimeLight.stopLimeLight();
        gamepadDriveTrainController.exit();
    }

    public void initSubsystems() {

        telemetry.setAutoClear(true);

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        visionLimeLight = new VisionLimeLight(hardwareMap, telemetry);
        telemetry.addLine("Vision Initialized");
        telemetry.update();

        intakeArm = new IntakeArm(hardwareMap, telemetry);
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        intakeSlides = new IntakeSlides(hardwareMap, telemetry);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        outtake = new Outtake(hardwareMap, telemetry);
        telemetry.addLine("Outtake Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new TestGamepadController(gamepad1, gamepad2, driveTrain, telemetry);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadDriveTrainController = new GamepadDriveTrainController(gamepad1, driveTrain, this);
        telemetry.addLine("Gamepad DriveTrain Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();

    }

    public void selectColor() {
        while (!isStopRequested()) {
            telemetry.addLine("Select Color to pick:");
            telemetry.addLine("---------------------------------------");
            telemetry.addData("    Yellow ", "(B / O)");
            telemetry.addData("    Red ", "(Y / Δ)");
            telemetry.addData("    Blue", "(A / X)");
            if (gamepadController.gp1GetCirclePress()) {
                selectedTargetColor = ColorRange.YELLOW;
                break;
            }
            if (gamepadController.gp1GetTrianglePress()) {
                selectedTargetColor = ColorRange.RED;
                break;
            }
            if (gamepadController.gp1GetCrossPress()) {
                selectedTargetColor = ColorRange.BLUE;;
                break;
            }

            telemetry.update();
        }
    }

    public void printDebugMessages() {
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start", "");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat TestVision");
            visionLimeLight.printDebugMessages();
            intakeSlides.printDebugMessages();
            intakeArm.printDebugMessages();
            driveTrain.printDebugMessages();

        }
        telemetry.update();
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}