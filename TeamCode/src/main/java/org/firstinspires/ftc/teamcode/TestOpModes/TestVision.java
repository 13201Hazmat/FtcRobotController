package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

@TeleOp (name = "Vision", group = "Testing")
public class TestVision extends LinearOpMode {
    public Vision vision;
    public VisionPortal visionPortal;
    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;

    ColorRange selectedTargetColor = ColorRange.BLUE;

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;

        initSubsystems();

        selectColor();

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            vision.locateNearestSamplefromRobot(selectedTargetColor);

            printDebugMessages();
        }
    }

    public void initSubsystems() {

        telemetry.setAutoClear(false);

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        vision = new Vision(hardwareMap, telemetry);
        telemetry.addLine("Vision Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new TestGamepadController(gamepad1, gamepad2, driveTrain, telemetry);
        telemetry.addLine("Gamepad Initialized");
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
            vision.printDebugMessages();

            for (ColorBlobLocatorProcessor.Blob b : vision.blobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }

        }
        telemetry.update();
    }
}


