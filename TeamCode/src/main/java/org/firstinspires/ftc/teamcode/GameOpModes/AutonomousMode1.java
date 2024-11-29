
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
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.Controllers.SpecimenController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.SpecimenHandler;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "Hazmat Autonomous Mode 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousMode1 extends LinearOpMode {

    public GamepadController gamepadController;
    public SpecimenController specimenController;
    public IntakeController intakeController;
    public OuttakeController outtakeController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public SpecimenHandler specimenHandler;
    public Climber climber;

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
    //LEFT
    Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0)); // Starting Pose
    Pose2d submersibleSpecimen = new Pose2d(0, 0, Math.toRadians(0));
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
            trajColorSampleThreeToSpecimenPickup, trajSpecimenPickupToSubmerssible,
            trajSubmerssibleToSpecimenPickup, trajSubmerssibleToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;
        drive = new MecanumDrive(hardwareMap, initPose);

        switch (GameField.startPosition) {
            case LEFT:
                submersibleSpecimen = new Pose2d(28, -11, Math.toRadians(0));
                bucket = new Pose2d(7, 41, Math.toRadians(-45));
                yellowSampleOne = new Pose2d(7.5, 41, Math.toRadians(-15));
                yellowSampleTwo = new Pose2d(7.5, 41, Math.toRadians(3));
                yellowSampleThree = new Pose2d(7.5, 41, Math.toRadians(23)); //TODO set values
                submersiblePark = new Pose2d(60, 6.5, Math.toRadians(-90));
                break;

            case RIGHT:
                submersibleSpecimen = new Pose2d(28, 11, Math.toRadians(0));
                specimenPickup = new Pose2d(-13.00, 14.96, Math.toRadians(-180));
                colorSampleOne = new Pose2d(0, 9.97, Math.toRadians(-1.28));
                colorSampleTwo = new Pose2d(2, 9.97, Math.toRadians(-1.28));
                colorSampleThree = new Pose2d(4, 9.97, Math.toRadians(-1.28));
                observationPark = new Pose2d(-25.71, 6.34, Math.toRadians(0));
                break;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        if (GameField.startPosition == GameField.START_POSITION.LEFT) {
            //move to submersible
            trajInitToSubmerssible = drive.actionBuilder(initPose)
                    .strafeTo(submersibleSpecimen.position)
                    //.strafeToLinearHeading(submersibleSpecimen.position, submersiblePark.heading)
                    .build();

            trajSubmerssibleToBucket = drive.actionBuilder(submersibleSpecimen)
                    .setReversed(true)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    .build();

            trajInitToBucket = drive.actionBuilder(initPose)
                    .strafeToLinearHeading(bucket.position, bucket.heading)
                    //.strafeToLinearHeading(submersibleSpecimen.position, submersiblePark.heading)
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
            //move to submersible
            trajInitToSubmerssible = drive.actionBuilder(initPose)
                    .strafeTo(submersibleSpecimen.position)
                    .build();

            trajSubmerssibleToColorSampleOne = drive.actionBuilder(submersibleSpecimen)
                    .setReversed(true)
                    .splineTo(colorSampleOne.position,colorSampleOne.heading)
                    .build();

            trajColorSampleOneToColorSampleTwo = drive.actionBuilder(colorSampleOne)
                    .strafeToLinearHeading(colorSampleTwo.position, colorSampleTwo.heading)
                    .build();

            trajColorSampleTwoToColorSampleThree = drive.actionBuilder(colorSampleTwo)
                    .strafeToLinearHeading(colorSampleThree.position, colorSampleThree.heading)
                    .build();

            trajColorSampleThreeToSpecimenPickup = drive.actionBuilder(colorSampleTwo)
                    .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                    .build();

            trajSpecimenPickupToSubmerssible = drive.actionBuilder(specimenPickup)
                    .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                    .build();

            trajSubmerssibleToSpecimenPickup = drive.actionBuilder(submersibleSpecimen)
                    .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                    .build();

            trajSubmerssibleToObservationPark = drive.actionBuilder(submersibleSpecimen)
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
                                specimenController.closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER),
                                trajInitToSubmerssible,
                                new SleepAction(1),
                                //hang specimen
                                specimenController.latchAndOpenGripAndMoveToPickupAction(),
                                //intakeController.IntakeSampleAtStartAction(),
                                trajSubmerssibleToBucket,
                                new SleepAction(1)
                        )
                );
            } else {
                Actions.runBlocking(
                        new SequentialAction(
                                new SleepAction(intialWaitTime),
                                trajInitToBucket,
                                new SleepAction(1)
                        )
                );
            }

            Actions.runBlocking(
                    new SequentialAction(
                            //drop sample at high bucket
                            // outtakeController.placeSampleInHighBucketAction(),
                            //lower to transfer
                            //outtakeController.lowerToTransferAction(),
                            trajBucketToYellowSampleOne,
                            new SleepAction(1),
                            //intake sample
                            //intakeController.IntakeSampleAtYS1Action(),
                            trajYellowSampleOneToBucket,
                            new SleepAction(1),
                            //drop sample at high bucket
                            //outtakeController.placeSampleInHighBucketAction(),
                            //lower to transfer
                            //outtakeController.lowerToTransferAction(),
                            trajBucketToYellowSampleTwo,
                            new SleepAction(1),
                            //intake sample
                            //intakeController.IntakeSampleAtYS2Action(),
                            trajYellowSampleTwoToBucket,
                            new SleepAction(1),
                            //drop sample at high bucket
                            //outtakeController.placeSampleInHighBucketAction(),
                            //lower to transfer
                            //outtakeController.lowerToTransferAction(),
                            trajBucketToYellowSampleThree,
                            new SleepAction(1),
                            //intake sample
                            //intakeController.IntakeSampleAtYS2Action(),
                            trajYellowSampleThreeToBucket,
                            new SleepAction(1),
                            //drop sample at high bucket
                            //outtakeController.placeSampleInHighBucketAction(),
                            //lower to transfer
                            //outtakeController.lowerToTransferAction(),
                            //TODO once tested and adjusted, add sample 3 code
                            trajBucketToSubmerssiblePark,
                            new SleepAction(1)

                    )
            );
        } else { //autoOption == RIGHT
            Actions.runBlocking(
                    new SequentialAction(
                            //Drop prelaod specimen to high chamber
                            new SleepAction(intialWaitTime),
                            specimenController.closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER),
                            trajInitToSubmerssible,
                            new SleepAction(1),
                            //hang specimen
                            specimenController.latchAndOpenGripAndMoveToPickupAction(),
                            //pick up sample from submerssible
                            intakeController.IntakeSampleAtStartAction(),
                            //Move to Sample 1 pickup
                            trajSubmerssibleToColorSampleOne,
                            new SleepAction(1),
                            //Drop sample picked up from submerssible to observation
                            outtakeController.dropSampleToObservationZoneAction(),
                            new SleepAction(1),
                            //Pick color Sample 1 and drop in Observation Zone
                            //intakeController.IntakeSampleOneAction(),
                            new SleepAction(1),
                            outtakeController.dropSampleToObservationZoneAction(),
                            new SleepAction(1),
                            //Pick color Sample 2 and drop in Observation Zone
                            trajColorSampleOneToColorSampleTwo,
                            //intakeController.IntakeSampleTwoAction(),
                            new SleepAction(1),
                            outtakeController.dropSampleToObservationZoneAction(),
                            new SleepAction(1),
                            //Pick color Sample 3 and drop in Observation Zone
                            trajColorSampleTwoToColorSampleThree,
                            //intakeController.IntakeSampleThreeAction(),
                            new SleepAction(1),
                            outtakeController.dropSampleToObservationZoneAction(),
                            new SleepAction(1),
                            //Move to Specimen 0 Pickup
                            trajColorSampleThreeToSpecimenPickup,
                            new SleepAction(1),
                            specimenController.closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssible,
                            new SleepAction(1),
                            specimenController.latchAndOpenGripAndMoveToPickupAction(),
                            trajSubmerssibleToSpecimenPickup,
                            new SleepAction(1),
                            specimenController.closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssible,
                            new SleepAction(1),
                            specimenController.latchAndOpenGripAndMoveToPickupAction(),
                            trajSubmerssibleToSpecimenPickup,
                            new SleepAction(1),
                            specimenController.closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER),
                            trajSpecimenPickupToSubmerssible,
                            new SleepAction(1),
                            specimenController.latchAndOpenGripAndMoveToPickupAction(),
                            trajSubmerssibleToSpecimenPickup,
                            new SleepAction(1),
                            specimenController.closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH),
                            trajSubmerssibleToObservationPark,
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
                break;
            }
            if (gamepadController.gp1GetCrossPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
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
        intakeController = new IntakeController(intakeArm, intakeSlides, this);
        telemetry.addLine("IntakeController Initialized");
        telemetry.update();

        outtake = new Outtake(hardwareMap, telemetry);
        telemetry.addLine("Outtake Initialized");
        telemetry.update();

        outtakeController = new OuttakeController(outtake, this);
        telemetry.addLine("Outtake Controller Initialized");
        telemetry.update();

        specimenHandler = new SpecimenHandler(hardwareMap, telemetry);
        telemetry.addLine("SpecimenHandler Initialized");
        telemetry.update();

        specimenController = new SpecimenController(specimenHandler, this);
        telemetry.addLine("Specimen Controller Initialized");
        telemetry.update();

        climber = new Climber(hardwareMap, telemetry);
        telemetry.addLine("Climber Initialized");
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

        gamepadController = new GamepadController(gamepad1, gamepad2, intakeArm, intakeSlides, intakeController,
                outtake, outtakeController, specimenHandler, specimenController, climber, telemetry, this);
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