
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
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Hazmat Autonomous
 */

@Autonomous(name = "Hazmat Auto RIGHT 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousRightSpecimen1 extends LinearOpMode {

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
    Pose2d pickupSpecimenPreload2 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenPreload2 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d postSpecimenPreload = new Pose2d(0,0,Math.toRadians(0));
    Pose2d submersibleSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenFour = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d prePickupSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d prePickupSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d prePickupSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenFour = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark2 = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToSubmersiblePreload,
            trajSubmersiblePreloadToSpecimenPreload2, trajSpecimenPreload2ToSubmersiblePreload2,
            trajSubmersiblePreload2ToColorSampleNear, trajColorSampleNearToObservationDrop,
            trajObservationDropToColorSampleMiddle, trajColorSampleMiddleToObservationDrop,
            trajObservationDropToColorSampleFar, trajColorSampleFarToObservationDrop,
            trajObservationDropToPickupSpecimenOne,
            trajPickupSpecimenOneToSubmersibleOne, trajSubmersibleOneToPickupSpecimenTwo,
            trajPickupSpecimenTwoToSubmersibleTwo, trajSubmersibleTwoToPickupSpecimenThree,
            trajPickupSpecimenThreeToSubmersibleThree, trajSubmersibleThreeToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        drive = new MecanumDrive(hardwareMap, initPose);
        submersibleSpecimenPreload = new Pose2d(-31.5, -1.2, Math.toRadians(0));
        //postSpecimenPreload = new Pose2d(-20, 28, Math.toRadians(121));
        pickupSpecimenPreload2 = new Pose2d(-7.3, 23.3, Math.toRadians(60));
        submersibleSpecimenPreload2 =  new Pose2d(-26.7, -13, Math.toRadians(0));
        colorSampleNear = new Pose2d(-17, 26.7, Math.toRadians(162.5));//-26
        colorSampleMiddle = new Pose2d(-17, 26.7, Math.toRadians(145));//121
        colorSampleFar = new Pose2d(-17, 26.7, Math.toRadians(140));
        observationDrop = new Pose2d(-17, 26.7, Math.toRadians(45));
        //prePickupSpecimenOne = new Pose2d(4, -28, Math.toRadians(-175)); //-180
        pickupSpecimenOne = new Pose2d(-17, 26.7, Math.toRadians(10)); //-180
        submersibleSpecimenOne = new Pose2d(-29.7, -4.8, Math.toRadians(0));
        //prePickupSpecimenTwo = new Pose2d(8, -28, Math.toRadians(-175)); //-180
        pickupSpecimenTwo = new Pose2d(-7.3, 10.3, Math.toRadians(75)); //-180
        submersibleSpecimenTwo = new Pose2d(-30.7, -9, Math.toRadians(0));
        //prePickupSpecimenThree = new Pose2d(8, -28, Math.toRadians(-175)); //-180
        pickupSpecimenThree = new Pose2d(-7.3, 10.3, Math.toRadians(60)); //-180
        submersibleSpecimenThree = new Pose2d(-26.7, -11, Math.toRadians(0));
        //pickupSpecimenFour = new Pose2d(-7.3, 23.3, Math.toRadians(60)); //-180
        //submersibleSpecimenFour = new Pose2d(-26.7, -13, Math.toRadians(0));
        observationPark = new Pose2d(-7.3, 7.4, Math.toRadians(70));

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToSubmersiblePreload = drive.actionBuilder(initPose)
                //.setTangent(180)
                //.splineToLinearHeading(submersibleSpecimenPreload, Math.toRadians(180))
                .strafeToLinearHeading(submersibleSpecimenPreload.position, submersibleSpecimenPreload.heading)
                .build();

        trajSubmersiblePreloadToSpecimenPreload2 = drive.actionBuilder(submersibleSpecimenPreload)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(pickupSpecimenPreload2, Math.toRadians(-90))
                .build();

        trajSpecimenPreload2ToSubmersiblePreload2 = drive.actionBuilder(pickupSpecimenPreload2)
                //.setReversed(false)
                .splineToLinearHeading(submersibleSpecimenPreload2, Math.toRadians(90))
                .build();

        trajSubmersiblePreload2ToColorSampleNear = drive.actionBuilder(submersibleSpecimenPreload2)
                .setReversed(false)
                .splineToLinearHeading(colorSampleNear, Math.toRadians(90))
                .build();

        trajColorSampleNearToObservationDrop = drive.actionBuilder(colorSampleNear)
                .turnTo(observationDrop.heading)
                .build();

        trajObservationDropToColorSampleMiddle = drive.actionBuilder(observationDrop)
                .turnTo(colorSampleMiddle.heading)
                .build();

        trajColorSampleMiddleToObservationDrop = drive.actionBuilder(colorSampleMiddle)
                .turnTo(observationDrop.heading)
                .build();

        trajObservationDropToColorSampleFar = drive.actionBuilder(observationDrop)
                .turnTo(colorSampleFar.heading)
                .build();

        trajColorSampleFarToObservationDrop = drive.actionBuilder(colorSampleFar)
                .turnTo(observationDrop.heading)
                .build();

        trajObservationDropToPickupSpecimenOne = drive.actionBuilder(observationDrop)
                .turnTo(pickupSpecimenOne.heading)
                .build();

        trajPickupSpecimenOneToSubmersibleOne = drive.actionBuilder(pickupSpecimenOne)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenOne, Math.toRadians(-90))
                .build();

        trajSubmersibleOneToPickupSpecimenTwo = drive.actionBuilder(submersibleSpecimenOne)
                //.setReversed(false)
                .splineToLinearHeading(pickupSpecimenTwo, Math.toRadians(90))
                .build();

        trajPickupSpecimenTwoToSubmersibleTwo = drive.actionBuilder(pickupSpecimenTwo)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenTwo, Math.toRadians(-90))
                .build();

        trajSubmersibleTwoToPickupSpecimenThree = drive.actionBuilder(submersibleSpecimenTwo)
                //.setReversed(false)
                .splineToLinearHeading(pickupSpecimenThree, Math.toRadians(90))//81.5
                .build();

        trajPickupSpecimenThreeToSubmersibleThree = drive.actionBuilder(pickupSpecimenThree)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenThree, Math.toRadians(-90))
                .build();

        trajSubmersibleThreeToObservationPark = drive.actionBuilder(submersibleSpecimenThree)
                //.setReversed(false)
                .splineToLinearHeading(observationPark, Math.toRadians(57))
                .build();
    }

    public void runAutonomousMode() {
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(intialWaitTime),
                        //*** Drop Prelaod specimen
                        intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER),
                        new ParallelAction(
                                trajInitToSubmersiblePreload,
                                intakeOuttakeController.moveOuttakeToHighChamberAction()
                        ),
                        new SleepAction(0.3),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //*** Drop Specimen Preload to Pick and drop Specimen Preload 2
                        new ParallelAction(
                                trajSubmersiblePreloadToSpecimenPreload2,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1,0),
                                intakeOuttakeController.moveOuttakeToTransferAction()
                        ),
                        //new SleepAction(0.1),
                        new ParallelAction(
                                intakeOuttakeController.resetOuttakeSlidesTouchAction(),
                                intakeOuttakeController.pickupSequenceSpecimenAction()
                        ),
                        new SleepAction(0.1),
                        new ParallelAction(
                                new SequentialAction(
                                        intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                        intakeOuttakeController.moveOuttakeToHighChamberAction()
                                ),
                                trajSpecimenPreload2ToSubmersiblePreload2
                        ),
                        new SleepAction(0.2),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //*** Drop Specimen Preload 2 to Move Color Sample Near to Observation Zone
                        new ParallelAction(
                                trajSubmersiblePreload2ToColorSampleNear,
                                intakeOuttakeController.moveOuttakeToTransferAction(),
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.67, 30)
                        ),
                        //new SleepAction(0.6),
                        new ParallelAction(
                                intakeOuttakeController.resetOuttakeSlidesTouchAction(),
                                intakeOuttakeController.pickupSequenceAction()
                        ),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajColorSampleNearToObservationDrop,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1, 90)
                        ),
                        new SleepAction(0.4),
                        intakeOuttakeController.openIntakeGripAction(),

                        //*** Move Color Sample Middle to Observation Zone
                        new ParallelAction(
                                trajObservationDropToColorSampleMiddle,
                                intakeOuttakeController.moveOuttakeToTransferAction(),
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.67, 30)
                        ),
                        //new SleepAction(0.6),
                        intakeOuttakeController.pickupSequenceAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajColorSampleMiddleToObservationDrop,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1, 90)
                        ),
                        new SleepAction(0.4),
                        intakeOuttakeController.openIntakeGripAction(),

                        //*** Move Color Sample Far to Observation Zone
                        new ParallelAction(
                                trajObservationDropToColorSampleFar,
                                intakeOuttakeController.moveOuttakeToTransferAction(),
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.67, 30)
                        ),
                        //new SleepAction(0.6),
                        intakeOuttakeController.pickupSequenceAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajColorSampleFarToObservationDrop,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1, 90)
                        ),
                        new SleepAction(0.4),
                        intakeOuttakeController.openIntakeGripAction(),

                        //Observation Drop to Pick and drop Specimen 1
                        new ParallelAction(
                                trajObservationDropToPickupSpecimenOne,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.7, 0)
                        ),
                        //new SleepAction(0.1),
                        intakeOuttakeController.resetOuttakeSlidesTouchAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                new SequentialAction(
                                        intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                        intakeOuttakeController.moveOuttakeToHighChamberAction()
                                ),
                                trajPickupSpecimenOneToSubmersibleOne
                        ),
                        new SleepAction(0.2),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //Drop Specimen 1 to Pick and drop Specimen 2
                        new ParallelAction(
                                trajSubmersibleOneToPickupSpecimenTwo,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.7, 0)
                        ),
                        //new SleepAction(0.1),
                        new ParallelAction(
                                intakeOuttakeController.resetOuttakeSlidesTouchAction(),
                                intakeOuttakeController.pickupSequenceSpecimenAction()
                        ),
                        new SleepAction(0.1),
                        new ParallelAction(
                                new SequentialAction(
                                        intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                        intakeOuttakeController.moveOuttakeToHighChamberAction()
                                ),
                                trajPickupSpecimenTwoToSubmersibleTwo
                        ),
                        new SleepAction(0.2),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //Drop Specimen 2 to Pick and drop Specimen 3
                        new ParallelAction(
                                trajSubmersibleTwoToPickupSpecimenThree,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.7, 0)
                        ),
                        //new SleepAction(0.1),
                        new ParallelAction(
                                intakeOuttakeController.resetOuttakeSlidesTouchAction(),
                                intakeOuttakeController.pickupSequenceSpecimenAction()
                        ),
                        new SleepAction(0.1),
                        new ParallelAction(
                                new SequentialAction(
                                        intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                        intakeOuttakeController.moveOuttakeToHighChamberAction()
                                ),
                                trajPickupSpecimenThreeToSubmersibleThree
                        ),
                        new SleepAction(0.2),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //Submersible Three to Park
                        new ParallelAction(
                                trajSubmersibleThreeToObservationPark,
                                intakeOuttakeController.moveOuttakeToTransferAction(),
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1,0)
                        ),
                        intakeOuttakeController.resetOuttakeSlidesTouchAction(),
                        new SleepAction(0.1)
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
            //specimenHandler.printDebugMessages();
            //climber.printDebugMessages();
            //lights.printDebugMessages();
        }
        telemetry.update();
    }
}
