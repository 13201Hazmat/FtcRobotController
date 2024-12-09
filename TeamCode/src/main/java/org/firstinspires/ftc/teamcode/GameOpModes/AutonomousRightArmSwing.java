
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

package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeOuttakeController;
import org.firstinspires.ftc.teamcode.Controllers.SpecimenController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.SpecimenHandler;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Hazmat Autonomous
 */

@Autonomous(name = "Hazmat Auto RIGHT ARM SWING", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousRightArmSwing extends LinearOpMode {

    public GamepadController gamepadController;
    public SpecimenController specimenController;
    public IntakeOuttakeController intakeOuttakeController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public SpecimenHandler specimenHandler;
    public Climber climber;
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
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;
        GameField.startPosition = GameField.START_POSITION.RIGHT;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Key Pay inputs to selecting Starting Position of robot
        telemetry.addData("Selected Starting Position", GameField.startPosition);

        //Build trajectories
        buildAutonoumousMode();

        //lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("RIGHT AUTO");
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            //lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);

            runAutonomousMode();
        }
    }   // end runOpMode()

    //List All Poses

    Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0)); // Starting Pose
    Pose2d submersibleSpecimenPreload = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d postSpecimenPreload = new Pose2d(0,0,Math.toRadians(0));

    Pose2d postPreload = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenFour = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d specimenPickup = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToSubmerssible,
            trajSubmerssibleToColorSampleFar, trajColorSampleFartoObservationDrop,
            trajObservationDropToColorSampleMiddle, trajColorSampleMiddleToObservationDrop,
            trajObservationDropToColorSampleNear, trajColorSampleNearToObservationDrop,
            trajObservationDropToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleOne, trajSubmerssibleOneToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleTwo, trajSubmerssibleTwoToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleThree, trajSubmerssibleThreeToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleFour, trajSubmerssibleFourToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        drive = new MecanumDrive(hardwareMap, initPose);

        submersibleSpecimenPreload = new Pose2d(29, 4, Math.toRadians(0));
        postSpecimenPreload = new Pose2d(18, -15, Math.toRadians(-12));
        colorSampleFar = new Pose2d(17.75, -33, Math.toRadians(-51));
        colorSampleMiddle = new Pose2d(16, -27, Math.toRadians(-43));
        colorSampleNear = new Pose2d(16, -27, Math.toRadians(-30));
        observationDrop = new Pose2d(16, -27.5, Math.toRadians(-141));
        specimenPickup = new Pose2d(3, -28, Math.toRadians(-180));
        submersibleSpecimenOne = new Pose2d(27, 9, Math.toRadians(0));
        submersibleSpecimenTwo = new Pose2d(27, 12, Math.toRadians(0));
        submersibleSpecimenThree = new Pose2d(27, 15, Math.toRadians(0));
        submersibleSpecimenFour = new Pose2d(27, 18, Math.toRadians(0));
        observationPark = new Pose2d(9, -32, Math.toRadians(70));

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToSubmerssible = drive.actionBuilder(initPose)
                .splineTo(submersibleSpecimenPreload.position, submersibleSpecimenPreload.heading)
                .build();

        trajSubmerssibleToColorSampleFar = drive.actionBuilder(submersibleSpecimenPreload)
                .setReversed(true)
                .lineToX(postSpecimenPreload.position.x)
                .strafeToLinearHeading(colorSampleFar.position, colorSampleFar.heading)
                .build();

        trajColorSampleFartoObservationDrop = drive.actionBuilder(colorSampleFar)
                .strafeToLinearHeading(observationDrop.position, observationDrop.heading)
                .build();

        trajObservationDropToColorSampleMiddle = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(colorSampleMiddle.position, colorSampleMiddle.heading)
                .build();

        trajColorSampleMiddleToObservationDrop = drive.actionBuilder(colorSampleMiddle)
                .strafeToLinearHeading(observationDrop.position, observationDrop.heading)
                .build();

        trajObservationDropToColorSampleNear = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(colorSampleNear.position, colorSampleNear.heading)
                .build();


        trajColorSampleNearToObservationDrop = drive.actionBuilder(colorSampleNear)
                .strafeToLinearHeading(observationDrop.position, observationDrop.heading)
                .build();

        trajObservationDropToSpecimenPickup = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                .build();

        trajSpecimenPickupToSubmerssibleOne = drive.actionBuilder(specimenPickup)
                .strafeToLinearHeading(submersibleSpecimenOne.position, submersibleSpecimenOne.heading)
                .build();

        trajSubmerssibleOneToSpecimenPickup = drive.actionBuilder(submersibleSpecimenOne)
                .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                .build();

        trajSpecimenPickupToSubmerssibleTwo = drive.actionBuilder(specimenPickup)
                .strafeToLinearHeading(submersibleSpecimenTwo.position, submersibleSpecimenTwo.heading)
                .build();

        trajSubmerssibleTwoToSpecimenPickup = drive.actionBuilder(submersibleSpecimenTwo)
                .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                .build();

        trajSpecimenPickupToSubmerssibleThree = drive.actionBuilder(specimenPickup)
                .strafeToLinearHeading(submersibleSpecimenThree.position, submersibleSpecimenThree.heading)
                .build();

        trajSubmerssibleThreeToSpecimenPickup = drive.actionBuilder(submersibleSpecimenThree)
                .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                .build();

        trajSpecimenPickupToSubmerssibleFour = drive.actionBuilder(specimenPickup)
                .strafeToLinearHeading(submersibleSpecimenFour.position, submersibleSpecimenFour.heading)
                .build();

        trajSubmerssibleFourToObservationPark = drive.actionBuilder(submersibleSpecimenFour)
                .strafeTo(observationPark.position)
                .build();
    }

    public void runAutonomousMode() {
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(intialWaitTime),
                        specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                        trajInitToSubmerssible,
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleToColorSampleFar,
                        intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1, 80),
                        intakeOuttakeController.pickupSequenceAction(),
                        new ParallelAction(
                                trajColorSampleFartoObservationDrop,
                                intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.8)
                        ),
                        intakeOuttakeController.openIntakeGripAction(),
                        new ParallelAction(
                                trajObservationDropToColorSampleMiddle,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.8, 45)
                        ),
                        intakeOuttakeController.pickupSequenceAction(),
                        new ParallelAction(
                                trajColorSampleMiddleToObservationDrop,
                                intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.8)
                        ),
                        intakeOuttakeController.openIntakeGripAction(),
                        new ParallelAction(
                                trajObservationDropToColorSampleNear,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.8, 20)
                        ),
                        intakeOuttakeController.pickupSequenceAction(),
                        new ParallelAction(
                                trajColorSampleNearToObservationDrop,
                                intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.8)
                        ),
                        intakeOuttakeController.openIntakeGripAction(),
                        new ParallelAction(
                                intakeOuttakeController.moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED),
                                intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.SPECIMEN_PICKUP),
                                trajObservationDropToSpecimenPickup
                        ),
                        new ParallelAction(
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                                trajSpecimenPickupToSubmerssibleOne
                        ),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleOneToSpecimenPickup,
                        new ParallelAction(
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                                trajSpecimenPickupToSubmerssibleTwo
                        ),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleTwoToSpecimenPickup,
                        new ParallelAction(
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                                trajSpecimenPickupToSubmerssibleThree
                        ),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleThreeToSpecimenPickup,
                        new ParallelAction(
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                                trajSpecimenPickupToSubmerssibleFour
                        ),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH),
                        trajSubmerssibleFourToObservationPark,
                        new ParallelAction(
                                intakeOuttakeController.setToAutoEndStateObservationZoneParkAction(),
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH)
                        ),
                        new SleepAction(1)
                )
        );
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

        specimenHandler = new SpecimenHandler(hardwareMap, telemetry);
        telemetry.addLine("SpecimenHandler Initialized");
        telemetry.update();

        climber = new Climber(hardwareMap, telemetry);
        telemetry.addLine("Climber Initialized");
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

        specimenController = new SpecimenController(specimenHandler, this);
        telemetry.addLine("Specimen Controller Initialized");
        telemetry.update();

        gamepadController = new GamepadController(gamepad1, gamepad2, intakeArm, intakeSlides, intakeOuttakeController,
                outtake, specimenHandler, specimenController, climber, telemetry, this);
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
            specimenHandler.printDebugMessages();
            climber.printDebugMessages();
            //lights.printDebugMessages();
        }
        telemetry.update();
    }

    public Action printDebugMessagesAction(String position) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                printDebugMessages(position);
                return false;
            }
        };
    }


}