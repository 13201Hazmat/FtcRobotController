
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

@Autonomous(name = "Hazmat Auto RIGHT ARM SWING 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousRightArmSwing1 extends LinearOpMode {

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
    Pose2d pickupSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenFour = new Pose2d(0, 0, Math.toRadians(0));
    //Pose2d colorSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToSubmerssiblePreload, trajSubmerssiblePreloadToPostPreload,
            trajPostPreloadToColorSampleMiddle,  trajColorSampleMiddleToObservationDrop,
            trajObservationDropToColorSampleNear, trajColorSampleNearToObservationDrop,
            trajObservationDropToPickupSpecimenOne,
            trajPickupSpecimenOneToSubmerssibleOne, trajSubmerssibleOneToPickupSpecimenTwo,
            trajPickupSpecimenTwoToSubmerssibleTwo, trajSubmerssibleTwoToPickupSpecimenThree,
            trajPickupSpecimenThreeToSubmerssibleThree, trajSubmerssibleThreeToPickupSpecimenFour,
            trajPickupSpecimenFourToSubmerssibleFour, trajSubmerssibleFourToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        drive = new MecanumDrive(hardwareMap, initPose);

        submersibleSpecimenPreload = new Pose2d(31, 4, Math.toRadians(0));
        postSpecimenPreload = new Pose2d(18, -15, Math.toRadians(-44));
        //colorSampleFar = new Pose2d(17.75, -33, Math.toRadians(-51));
        colorSampleMiddle = new Pose2d(17, -28, Math.toRadians(-44));
        colorSampleNear = new Pose2d(16, -27, Math.toRadians(-28));
        observationDrop = new Pose2d(16, -27.5, Math.toRadians(-141));
        pickupSpecimenOne = new Pose2d(0, -27, Math.toRadians(-185)); //-180
        submersibleSpecimenOne = new Pose2d(27, 9, Math.toRadians(0));
        pickupSpecimenTwo = new Pose2d(0, -27, Math.toRadians(-180)); //-180
        submersibleSpecimenTwo = new Pose2d(27, 12, Math.toRadians(0));
        pickupSpecimenThree = new Pose2d(0, -27, Math.toRadians(-180)); //-180
        submersibleSpecimenThree = new Pose2d(27, 15, Math.toRadians(0));
        pickupSpecimenFour = new Pose2d(0, -27, Math.toRadians(-180)); //-180
        submersibleSpecimenFour = new Pose2d(27, 18, Math.toRadians(0));
        observationPark = new Pose2d(4, -35, Math.toRadians(60));

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToSubmerssiblePreload = drive.actionBuilder(initPose)
                .splineTo(submersibleSpecimenPreload.position, submersibleSpecimenPreload.heading)
                .build();

        trajSubmerssiblePreloadToPostPreload = drive.actionBuilder(submersibleSpecimenPreload)
                .setReversed(true)
                .strafeToLinearHeading(postSpecimenPreload.position, postSpecimenPreload.heading)
                .build();

        trajPostPreloadToColorSampleMiddle = drive.actionBuilder(postSpecimenPreload)
                .strafeToLinearHeading(colorSampleMiddle.position, colorSampleMiddle.heading)
                .build();

        /*
        trajColorSampleFartoObservationDrop = drive.actionBuilder(colorSampleFar)
                .strafeToLinearHeading(observationDrop.position, observationDrop.heading)
                .build();

        trajObservationDropToColorSampleMiddle = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(colorSampleMiddle.position, colorSampleMiddle.heading)
                .build();

         */
        trajColorSampleMiddleToObservationDrop = drive.actionBuilder(colorSampleMiddle)
                .strafeToLinearHeading(observationDrop.position, observationDrop.heading)
                .build();

        trajObservationDropToColorSampleNear = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(colorSampleNear.position, colorSampleNear.heading)
                .build();


        trajColorSampleNearToObservationDrop = drive.actionBuilder(colorSampleNear)
                .strafeToLinearHeading(observationDrop.position, observationDrop.heading)
                .build();

        trajObservationDropToPickupSpecimenOne = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

        trajPickupSpecimenOneToSubmerssibleOne = drive.actionBuilder(pickupSpecimenOne)
                .strafeToLinearHeading(submersibleSpecimenOne.position, submersibleSpecimenOne.heading)
                .build();

        trajSubmerssibleOneToPickupSpecimenTwo = drive.actionBuilder(submersibleSpecimenOne)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

        trajPickupSpecimenTwoToSubmerssibleTwo = drive.actionBuilder(pickupSpecimenOne)
                .strafeToLinearHeading(submersibleSpecimenTwo.position, submersibleSpecimenTwo.heading)
                .build();

        trajSubmerssibleTwoToPickupSpecimenThree = drive.actionBuilder(submersibleSpecimenTwo)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

        trajPickupSpecimenThreeToSubmerssibleThree = drive.actionBuilder(pickupSpecimenOne)
                .strafeToLinearHeading(submersibleSpecimenThree.position, submersibleSpecimenThree.heading)
                .build();

        trajSubmerssibleThreeToPickupSpecimenFour = drive.actionBuilder(submersibleSpecimenThree)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

        trajPickupSpecimenFourToSubmerssibleFour = drive.actionBuilder(pickupSpecimenOne)
                .strafeToLinearHeading(submersibleSpecimenFour.position, submersibleSpecimenFour.heading)
                .build();

        trajSubmerssibleFourToObservationPark = drive.actionBuilder(submersibleSpecimenFour)
                .turn(Math.toRadians(60))
                .strafeToLinearHeading(observationPark.position, observationPark.heading)
                .build();
    }

    public void runAutonomousMode() {
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(intialWaitTime),
                        specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                        trajInitToSubmerssiblePreload,
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        //trajSubmerssibleToColorSampleFar,
                        trajSubmerssiblePreloadToPostPreload,
                        intakeOuttakeController.extendIntakeArmByExtensionFactorAction(1),
                        trajPostPreloadToColorSampleMiddle,
                        intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1, 45),
                        /*intakeOuttakeController.pickupSequenceAction(),
                        new ParallelAction(
                                trajColorSampleFartoObservationDrop,
                                intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.8)
                        ),
                        intakeOuttakeController.openIntakeGripAction(),
                        new ParallelAction(
                                trajObservationDropToColorSampleMiddle,
                                intakeOuttakeController.extendIntakeArmByExtensionFactorAction(0.8, 45)
                        ),
                         */
                        new SleepAction(0.2),
                        intakeOuttakeController.pickupSequenceAction(),
                        new SleepAction(0.2),
                        trajColorSampleMiddleToObservationDrop,
                        new SleepAction(0.3),
                        intakeOuttakeController.openIntakeGripAction(),
                        new SleepAction(0.2),
                        trajObservationDropToColorSampleNear,
                        new SleepAction(0.2),
                        intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.75, 20),
                        new SleepAction(0.2),
                        intakeOuttakeController.pickupSequenceAction(),
                        new SleepAction(0.2),
                        trajColorSampleNearToObservationDrop,
                        new SleepAction(0.3),
                        intakeOuttakeController.openIntakeGripAction(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                intakeOuttakeController.moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED),
                                intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.SPECIMEN_PICKUP)
                        ),
                        new SleepAction(0.2),
                        trajObservationDropToPickupSpecimenOne,
                        new SleepAction(0.2),
                        specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                        trajPickupSpecimenOneToSubmerssibleOne,
                        new SleepAction(0.2),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleOneToPickupSpecimenTwo,
                        new SleepAction(0.2),
                        specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                        trajPickupSpecimenTwoToSubmerssibleTwo,
                        new SleepAction(0.2),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleTwoToPickupSpecimenThree,
                        new SleepAction(0.2),
                        specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                        trajPickupSpecimenThreeToSubmerssibleThree,
                        new SleepAction(0.2),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                        trajSubmerssibleThreeToPickupSpecimenFour,
                        new SleepAction(0.2),
                        specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                        trajPickupSpecimenFourToSubmerssibleFour,
                        new SleepAction(0.2),
                        specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH),
                        trajSubmerssibleFourToObservationPark,
                        /*new ParallelAction(
                                intakeOuttakeController.setToAutoEndStateObservationZoneParkAction(),
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH)
                        ),*/
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