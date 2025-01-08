
/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeOuttakeController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Hazmat Autonomous
 */
@TeleOp(name = "Endurance Testing", group = " 01-Testing")
public class EnduranceTesting extends LinearOpMode {

    public GamepadController gamepadController;
    public IntakeOuttakeController intakeOuttakeController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public Vision vision;
    //public Lights lights;

    public MecanumDrive drive;
    public TelemetryPacket telemetryPacket = new TelemetryPacket();

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);
    public ElapsedTime startTimer = new ElapsedTime(MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;
        GameField.startPosition = GameField.START_POSITION.LEFT;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Build trajectories
        //buildAutonoumousMode();

        //lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Endurance Tests");
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            //lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine(" Press Triangle for running Outtake Cycling");
            telemetry.addData("Outtake Cycling Avg Time", triangleAvgTime);
            telemetry.addLine(" Press Square for running Intake Cycling");
            telemetry.addData("Intake Cycling Avg Time", squareAvgTime);
            telemetry.addLine(" Press Circle for running Full Pickup to Drop High Bucket Cycling");
            telemetry.addData("Pickup to Drop High Bucket Avg Time", circleAvgTime);
            telemetry.addLine(" Press Cross for running Full Pickup to Drop High Chamber Cycling");
            telemetry.addData("Pickup to Drop High Chamber Avg Time", crossAvgTime);
            telemetry.update();
            runEnduranceTests();
        }

    }   // end runOpMode()

    ElapsedTime triangleTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime crossTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime circleTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime squareTimer = new ElapsedTime(MILLISECONDS);

    int triangleCounter, crossCounter, circleCounter, squareCounter;
    double triangleAvgTime = 0;
    double crossAvgTime = 0;
    double circleAvgTime = 0;
    double squareAvgTime = 0;

    public void runEnduranceTests() {
        //Outtake Cycling
        if (gamepadController.gp1GetTrianglePress()) {
            triangleTimer.reset();

            for (triangleCounter = 1; triangleCounter < 11; triangleCounter++) {
                Actions.runBlocking(
                        new SequentialAction(
                                intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.POST_TRANSFER),
                                intakeOuttakeController.closeOuttakeGripAction(),
                                intakeOuttakeController.moveOuttakeHighBucketAction(),
                                intakeOuttakeController.safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.HIGH_BUCKET),
                                intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                intakeOuttakeController.moveOuttakeToTransferAction(),
                                intakeOuttakeController.safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.TRANSFER),
                                intakeOuttakeController.openOuttakeGripAction()
                        )
                );
                triangleAvgTime = triangleTimer.time()/ triangleCounter;
                telemetry.addData("Counter", triangleCounter);
                telemetry.addData("Outtake Cycling Avg Time", triangleAvgTime);
                telemetry.update();
            }
        }

        //Intake Cycling
        if (gamepadController.gp1GetSquarePress()) {
            squareTimer.reset();

            for (squareCounter = 1; squareCounter < 11; squareCounter++){
                Actions.runBlocking(
                        new SequentialAction(
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(squareCounter/10, -90 + (double) (squareCounter-1) * 20),
                                intakeOuttakeController.pickupSequenceAction(),
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                intakeOuttakeController.openIntakeGripAction()
                                )
                        );
                squareAvgTime = squareTimer.time()/ squareCounter;
                telemetry.addData("Counter", squareCounter);
                telemetry.addData("Intake Cycling Avg Time", squareAvgTime);
                telemetry.update();
            }
        }

        // Intake to High Bucket Transfer Cycling
        if (gamepadController.gp1GetCirclePress()) {
            circleTimer.reset();

            for (circleCounter = 1; circleCounter < 11; circleCounter++) {
                Actions.runBlocking(
                        new SequentialAction(
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(circleCounter/10, -90 + (double) (circleCounter-1) * 20),
                                intakeOuttakeController.pickupSequenceAction(),
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakePreDropAction(),
                                intakeOuttakeController.moveOuttakeHighBucketAction(),
                                intakeOuttakeController.safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.HIGH_BUCKET),
                                intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                intakeOuttakeController.moveOuttakeToTransferAction(),
                                intakeOuttakeController.safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.TRANSFER)
                        )
                );
                circleAvgTime = circleTimer.time()/ circleCounter;
                telemetry.addData("Counter", circleCounter);
                telemetry.addData("Pickup to Drop High Bucket Avg Time", circleAvgTime);
                telemetry.update();
            }
        }

        // Intake to High Chamber Cycling
        if (gamepadController.gp1GetCrossPress()) {
            crossTimer.reset();

            for (crossCounter = 1; crossCounter < 11; crossCounter++) {
                Actions.runBlocking(
                        new SequentialAction(
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(crossCounter/10, -90 + (double) (crossCounter-1) * 20),
                                intakeOuttakeController.pickupSequenceAction(),
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                intakeOuttakeController.moveOuttakeToHighChamberDropSpecimenAction(),
                                intakeOuttakeController.moveOuttakeToTransferAction()
                        )
                );
                crossAvgTime = crossTimer.time()/ crossCounter;
                telemetry.addData("Counter", crossCounter);
                telemetry.addData("Pickup to Drop High Chamber Avg Time", crossAvgTime);
                telemetry.update();
            }
        }
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public void initSubsystems(){

        telemetry.setAutoClear(true);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        outtake = new Outtake(hardwareMap, telemetry);
        telemetry.addLine("Outtake Initialized");
        telemetry.update();

        safeWaitSeconds(1);

        intakeArm = new IntakeArm(hardwareMap, telemetry);
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        intakeSlides = new IntakeSlides(hardwareMap, telemetry);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        vision = new Vision(hardwareMap, telemetry);
        telemetry.addLine("Vision Initialized");
        telemetry.update();

        /*
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();
        */

        /* Create Controllers */
        //gamepadDriveTrainController = new GamepadDriveTrainController(gamepad1, driveTrain, this);
        //telemetry.addLine("Gamepad DriveTrain Initialized");
        //telemetry.update();

        intakeOuttakeController = new IntakeOuttakeController(intakeArm, intakeSlides, outtake, vision,this);
        telemetry.addLine("IntakeController Initialized");
        telemetry.update();

        gamepadController = new GamepadController(gamepad1, gamepad2, intakeArm, intakeSlides, intakeOuttakeController,
                outtake, telemetry, this);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

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

    @SuppressLint("DefaultLocale")
    public String toStringPose2d(Pose2d pose){
        return String.format("(%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.log()));
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(String position){
        //telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addLine("Robot ready to start");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat TeleOpMode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            telemetry.addData("PoseEstimateString :", toStringPose2d(drive.pose));
            telemetry.addData("Position", position);

            driveTrain.printDebugMessages();
            intakeArm.printDebugMessages();
            intakeSlides.printDebugMessages();
            outtake.printDebugMessages();
            //lights.printDebugMessages();
        }
        telemetry.update();
    }
}