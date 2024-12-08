
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
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.vision.opencv.ColorRange;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "Hazmat Autonomous Mode 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousMode1 extends LinearOpMode {

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
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", GameField.startPosition);

        //Build trajectories
        buildAutonoumousMode();

        //lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Alliance         ", GameField.playingAlliance);
            telemetry.addData("Selected Starting Position", GameField.startPosition);
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

    //LEFT
    Pose2d preBucket = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d bucket = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d yellowSampleOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d yellowSampleTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d yellowSampleThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersiblePark = new Pose2d(0, 0, Math.toRadians(0));

    enum INITIAL_LEFT_ACTION {
        SPECIMEN_ON_CHAMBER,
        SAMPLE_ON_HIGH_BASKET
    }
    INITIAL_LEFT_ACTION initialLeftAction = INITIAL_LEFT_ACTION.SPECIMEN_ON_CHAMBER;

    //RIGHT
    Pose2d postPreload = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenFour = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d specimenPickup = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToSubmerssible, trajSubmerssibleToBucket,
            trajInitToBucket,
            trajBucketToYellowSampleOne, trajYellowSampleOneToBucket,
            trajBucketToYellowSampleTwo, trajYellowSampleTwoToBucket,
            trajBucketToYellowSampleThree, trajYellowSampleThreeToBucket,
            trajBucketToSubmerssiblePark;

    Action trajSubmerssibleToColorSampleOne, trajColorSampleOneToColorSampleTwo, trajColorSampleTwoToColorSampleThree,
            trajColorSampleThreeToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleOne, trajSubmerssibleOneToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleTwo, trajSubmerssibleTwoToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleThree, trajSubmerssibleThreeToSpecimenPickup,
            trajSpecimenPickupToSubmerssibleFour, trajSubmerssibleFourToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;
        drive = new MecanumDrive(hardwareMap, initPose);

        switch (GameField.startPosition) {
            case LEFT:
                submersibleSpecimenPreload = new Pose2d(29, -12, Math.toRadians(0));
                preBucket= new Pose2d(29, -12, Math.toRadians(0));
                bucket = new Pose2d(9, 40.5, Math.toRadians(-45));
                yellowSampleOne = new Pose2d(11.5, 39, Math.toRadians(-15));
                yellowSampleTwo = new Pose2d(10.2, 39, Math.toRadians(4.5));
                yellowSampleThree = new Pose2d(13.2, 40.5, Math.toRadians(23)); //TODO set values
                submersiblePark = new Pose2d(59, 4, Math.toRadians(-90));
                break;

            case RIGHT:
                submersibleSpecimenPreload = new Pose2d(29, 4, Math.toRadians(0));
                postPreload =  new Pose2d(18, -30, Math.toRadians(0));
                colorSampleOne = new Pose2d(17.75, -42, Math.toRadians(-23.5));
                colorSampleTwo = new Pose2d(17, -41.5, Math.toRadians(0));
                colorSampleThree = new Pose2d(17.5, -41, Math.toRadians(25));
                specimenPickup = new Pose2d(3, -28, Math.toRadians(-180));
                submersibleSpecimenOne = new Pose2d(27, 9, Math.toRadians(0));
                submersibleSpecimenTwo = new Pose2d(27, 12, Math.toRadians(0));
                submersibleSpecimenThree = new Pose2d(27, 15, Math.toRadians(0));
                submersibleSpecimenFour = new Pose2d(27, 18, Math.toRadians(0));
                observationPark = new Pose2d(9, -32, Math.toRadians(70));
                break;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToSubmerssible = drive.actionBuilder(initPose)
                .splineTo(submersibleSpecimenPreload.position, submersibleSpecimenPreload.heading)
                .build();

        if (GameField.startPosition == GameField.START_POSITION.LEFT) {
            trajSubmerssibleToBucket = drive.actionBuilder(submersibleSpecimenPreload)
                    .setReversed(true)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    .build();

            trajInitToBucket = drive.actionBuilder(initPose)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    .build();

            //move to yellow sample one
            trajBucketToYellowSampleOne = drive.actionBuilder(bucket)
                    .strafeToLinearHeading(yellowSampleOne.position, yellowSampleOne.heading)
                    .build();

            trajYellowSampleOneToBucket = drive.actionBuilder(yellowSampleOne)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    .build();

            trajBucketToYellowSampleTwo = drive.actionBuilder(bucket)
                    .strafeToLinearHeading(yellowSampleTwo.position, yellowSampleTwo.heading)
                    .build();

            trajYellowSampleTwoToBucket = drive.actionBuilder(yellowSampleTwo)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    .build();

            trajBucketToYellowSampleThree = drive.actionBuilder(bucket)
                    .strafeToLinearHeading(yellowSampleThree.position, yellowSampleThree.heading)
                    .build();

            trajYellowSampleThreeToBucket = drive.actionBuilder(yellowSampleThree)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    .build();

            trajBucketToSubmerssiblePark = drive.actionBuilder(bucket)
                    .splineTo(submersiblePark.position, submersiblePark.heading)
                    //.strafeToLinearHeading(submersiblePark.position, submersiblePark.heading)
                    .build();

        } else { //autoOption == RIGHT
            trajSubmerssibleToColorSampleOne = drive.actionBuilder(submersibleSpecimenPreload)
                    .setReversed(true)
                    .strafeToLinearHeading(postPreload.position, postPreload.heading)
                    .strafeToLinearHeading(colorSampleOne.position,colorSampleOne.heading)
                    .build();

            trajColorSampleOneToColorSampleTwo = drive.actionBuilder(colorSampleOne)
                    .strafeToLinearHeading(colorSampleTwo.position, colorSampleTwo.heading)
                    .build();

            trajColorSampleTwoToColorSampleThree = drive.actionBuilder(colorSampleTwo)
                    .strafeToLinearHeading(colorSampleThree.position, colorSampleThree.heading)
                    .build();

            trajColorSampleThreeToSpecimenPickup = drive.actionBuilder(colorSampleThree)
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
                    .setReversed(true)
                    .strafeTo(observationPark.position)
                    .build();
        }
    }

    public void runAutonomousMode() {
        if (GameField.startPosition == GameField.START_POSITION.LEFT) {

            if (initialLeftAction == INITIAL_LEFT_ACTION.SPECIMEN_ON_CHAMBER) {
                Actions.runBlocking(
                        new SequentialAction(
                                new SleepAction(intialWaitTime),
                                //specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                                trajInitToSubmerssible,
                                new SleepAction(1),
                                //specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                                trajSubmerssibleToBucket,
                                new SleepAction(1)
                        )
                );
            } else {
                Actions.runBlocking(
                        new SequentialAction(
                                new SleepAction(intialWaitTime),
                                trajInitToBucket,
                                new SleepAction(1)/*,
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakePreDropAction(),
                                intakeOuttakeController.moveOuttakeToAction(Outtake.SLIDE_STATE.HIGH_BUCKET),
                                intakeOuttakeController.dropSamplefromOuttakeAction()*/
                        )
                );
            }


            Actions.runBlocking(
                    new SequentialAction(
                            trajBucketToYellowSampleOne,
                            new SleepAction(1),
                            //intakeOuttakeController.pickSampleAction(ColorRange.YELLOW),
                            trajYellowSampleOneToBucket,
                            new SleepAction(1),
                            //intakeOuttakeController.dropHighBucketAction(),
                            trajBucketToYellowSampleTwo,
                            new SleepAction(1),
                            //intakeOuttakeController.pickSampleAction(ColorRange.YELLOW),
                            trajYellowSampleTwoToBucket,
                            new SleepAction(1),
                            //intakeOuttakeController.dropHighBucketAction(),
                            trajBucketToYellowSampleThree,
                            new SleepAction(1),
                            //intakeOuttakeController.pickSampleAction(ColorRange.YELLOW),
                            trajYellowSampleThreeToBucket,
                            new SleepAction(1),
                            //intakeOuttakeController.dropHighBucketAction(),
                            //intakeOuttakeController.setToAutoEndStateAction(),
                            trajBucketToSubmerssiblePark,
                            //specimenController.moveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH),
                            new SleepAction(1)

                    )
            );
        } else { //autoOption == RIGHT
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(intialWaitTime),
                            //specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                            trajInitToSubmerssible,
                            new SleepAction(1),
                            //specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                            trajSubmerssibleToColorSampleOne,
                            new SleepAction(1),
                            //intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.13/0.2),
                            //intakeOuttakeController.pickSampleAction(GameField.allianceColor),
                            trajColorSampleOneToColorSampleTwo,
                            //intakeOuttakeController.dropSamplefromOuttakeAction(),
                            new SleepAction(1),
                            //intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.1/0.2),
                            //intakeOuttakeController.pickSampleAction(GameField.allianceColor),
                            trajColorSampleTwoToColorSampleThree,
                            //intakeOuttakeController.dropSamplefromOuttakeAction(),
                            new SleepAction(1),
                            //intakeOuttakeController.extendIntakeArmToPrePickupByExtensionFactorAction(0.13/0.2),
                            //intakeOuttakeController.pickSampleAction(GameField.allianceColor),
                            //intakeOuttakeController.dropSamplefromOuttakeAction(),
                            trajColorSampleThreeToSpecimenPickup,
                            new SleepAction(1),
                            //specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssibleOne,
                            new SleepAction(1),
                            //specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                            trajSubmerssibleOneToSpecimenPickup,
                            new SleepAction(1),
                            //specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssibleTwo,
                            new SleepAction(1),
                            //specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.PICKUP),
                            trajSubmerssibleTwoToSpecimenPickup,
                            new SleepAction(1),
                            //specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssibleThree,
                            new SleepAction(1),
                            //specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH),
                            trajSubmerssibleThreeToSpecimenPickup,
                            new SleepAction(1),
                            //specimenController.closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssibleFour,
                            new SleepAction(1),
                            //specimenController.latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH),
                            trajSubmerssibleFourToObservationPark,
                            intakeOuttakeController.setToAutoEndStateAction(),
                            new SleepAction(1)
                    )
            );
        }
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        //telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Alliance Color using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
            telemetry.addData("    Red ", "(Y / Δ)");
            telemetry.addData("    Blue", "(A / X)");
            if (gamepadController.gp1GetTrianglePress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                GameField.allianceColor = ColorRange.RED;
                break;
            }
            if (gamepadController.gp1GetCrossPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                GameField.allianceColor = ColorRange.BLUE;
                break;
            }

            telemetry.update();
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Alliance", GameField.playingAlliance);
            telemetry.addLine("Select Starting Position");
            telemetry.addData("    Left     ", "(X / ▢)");
            telemetry.addData("    Right    ", "(B / O)");

            if (gamepadController.gp1GetSquarePress()) {
                GameField.startPosition = GameField.START_POSITION.LEFT;
                break;
            }

            if (gamepadController.gp1GetCirclePress()) {
                GameField.startPosition = GameField.START_POSITION.RIGHT;
                break;
            }

            telemetry.update();
        }

        if (GameField.startPosition == GameField.START_POSITION.LEFT) {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Alliance", GameField.playingAlliance);
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addLine("Preloaded First Action : ");
                telemetry.addData("    Specimen on Chamber     ", "(Y / Δ)");
                telemetry.addData("    Sample on High Basket   ", "(A / X)");

                if (gamepadController.gp1GetTrianglePress()) {
                    initialLeftAction = INITIAL_LEFT_ACTION.SPECIMEN_ON_CHAMBER;
                    break;
                }

                if (gamepadController.gp1GetCrossPress()) {
                    initialLeftAction = INITIAL_LEFT_ACTION.SAMPLE_ON_HIGH_BASKET;
                    break;
                }

                telemetry.update();
            }
        }


        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Alliance", GameField.playingAlliance);
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addLine("Input Initial Wait time to start");
            telemetry.addLine("(Use dpad_up to increase, and dpad_down to decrease)");
            telemetry.addLine("(Press B / O  to finalize");
            telemetry.addData("    Initial Wait Time ", intialWaitTime);
            if (gamepadController.gp1GetDpad_upPress()) {
                intialWaitTime++;
            }
            if (gamepadController.gp1GetDpad_downPress()) {
                if (intialWaitTime != 0) intialWaitTime--;
            }
            if (gamepadController.gp1GetCirclePress()) {
                break;
            }
            telemetry.update();
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

        intakeArm = new IntakeArm(hardwareMap, telemetry);
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        intakeSlides = new IntakeSlides(hardwareMap, telemetry);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        outtake = new Outtake(hardwareMap, telemetry);
        telemetry.addLine("Outtake Initialized");
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
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat TeleOpMode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            telemetry.addData("PoseEstimateString :", toStringPose2d(drive.pose));

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


}