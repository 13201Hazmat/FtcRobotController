
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

package org.firstinspires.ftc.teamcode.GameOpModes.DontUse;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Disabled
@Autonomous(name = "Hazmat Auto RIGHT No Pick 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousRightSpecimenNoPickNEW extends LinearOpMode {

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

    public enum AUTO_OPTION {
        FIVE_SPECIMEN_AUTO,
        FOUR_SPECIMEN_AUTO
    }
    public AUTO_OPTION autoOption = AUTO_OPTION.FOUR_SPECIMEN_AUTO;


    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;
        GameField.startPosition = GameField.START_POSITION.RIGHT;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPositionAndPreload();
        telemetry.addData("Selected Starting Position", GameField.startPosition);
        telemetry.addData("Selected Auto Option", autoOption);

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
    Pose2d submersibleSpecimenPreloadDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenPreload2 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenPreload2 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenPreload2Drop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d postSpecimenPreload = new Pose2d(0,0,Math.toRadians(0));
    Pose2d submersibleSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenOneDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenTwoDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenThreeDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d preColorSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d preColorSampleNear1 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleNearDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d preColorSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleMiddleDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d preColorSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleFarDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToSubmersiblePreload, trajSubmersiblePreloadToSubmersiblePreloadDrop,
            trajSubmersiblePreloadDropToSpecimenPreload2, trajSpecimenPreload2ToSubmersiblePreload2, trajSubmersiblePreload2ToSubmersiblePreload2Drop,
            trajSubmersiblePreload2DropToPickupSpecimenOne,
            //trajSubmersiblePreload2DropToColorSampleNear, trajColorSampleNearToObservationDrop,
            //trajObservationDropToColorSampleMiddle, trajColorSampleMiddleToObservationDrop,
            //trajObservationDropToColorSampleFar, trajColorSampleFarToObservationDrop,
            //trajObservationDropToPickupSpecimenOne,
            trajPickupSpecimenOneToSubmersibleOne, trajSubmersibleOneToSubmersibleOneDrop, trajSubmersibleOneDropToPickupSpecimenTwo,
            trajPickupSpecimenTwoToSubmersibleTwo, trajSubmersibleTwoToSubmersibleTwoDrop, trajSubmersibleTwoDropToPickupSpecimenThree,
            trajPickupSpecimenThreeToSubmersibleThree, trajSubmersibleThreeToSubmersibleThreeDrop, trajSubmersibleThreeDropToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        drive = new MecanumDrive(hardwareMap, initPose);
        submersibleSpecimenPreload = new Pose2d(-31.5, 0, Math.toRadians(0));
        submersibleSpecimenPreloadDrop = new Pose2d(-31.5+6, 0, Math.toRadians(0));
        pickupSpecimenPreload2 = new Pose2d(0, 28, Math.toRadians(0));
        submersibleSpecimenPreload2 =  new Pose2d(-31.5, -5, Math.toRadians(0));
        submersibleSpecimenPreload2Drop =  new Pose2d(-31.5+6, -5, Math.toRadians(0));
        preColorSampleNear = new Pose2d(-25, 27, Math.toRadians(0));//-26
        preColorSampleNear1 = new Pose2d(-51, 27, Math.toRadians(0));//-26
        colorSampleNear = new Pose2d(-51, 36.5, Math.toRadians(0));//-26
        colorSampleNearDrop = new Pose2d(-10, 36.5, Math.toRadians(0));//-26
        preColorSampleMiddle = new Pose2d(-51, 36.5, Math.toRadians(0));//121
        colorSampleMiddle = new Pose2d(-51, 46, Math.toRadians(0));//121
        colorSampleMiddleDrop = new Pose2d(-10, 46, Math.toRadians(0));//121
        preColorSampleFar = new Pose2d(-51, 46, Math.toRadians(0));
        colorSampleFar = new Pose2d(-51, 51, Math.toRadians(0));
        colorSampleFarDrop = new Pose2d(-10, 51, Math.toRadians(0));
        observationDrop = new Pose2d(-17, 36.7, Math.toRadians(0));
        pickupSpecimenOne = pickupSpecimenPreload2; //new Pose2d(-17, 26.7, Math.toRadians(0)); //-180
        submersibleSpecimenOne = new Pose2d(-31.5, -4.8, Math.toRadians(0));
        submersibleSpecimenOneDrop = new Pose2d(-31.5+6, -4.8, Math.toRadians(0));
        pickupSpecimenTwo = pickupSpecimenOne; //new Pose2d(-7.3, 10.3, Math.toRadians(0)); //-180
        submersibleSpecimenTwo = new Pose2d(-31.5, -9, Math.toRadians(0));
        submersibleSpecimenTwoDrop = new Pose2d(-31.5+6, -9, Math.toRadians(0));
        pickupSpecimenThree = pickupSpecimenTwo; //new Pose2d(-7.3, 10.3, Math.toRadians(0)); //-180
        submersibleSpecimenThree = new Pose2d(-31.5, -11, Math.toRadians(0));
        submersibleSpecimenThreeDrop = new Pose2d(-31.5+6, -11, Math.toRadians(0));
        observationPark = new Pose2d(-7.3, 7.4, Math.toRadians(70));

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToSubmersiblePreload = drive.actionBuilder(initPose)
                //.setTangent(180)
                //.splineToLinearHeading(submersibleSpecimenPreload, Math.toRadians(180))
                .strafeToLinearHeading(submersibleSpecimenPreload.position, submersibleSpecimenPreload.heading)
                .build();

        trajSubmersiblePreloadToSubmersiblePreloadDrop = drive.actionBuilder(submersibleSpecimenPreload)
                .strafeToLinearHeading(submersibleSpecimenPreloadDrop.position, submersibleSpecimenPreloadDrop.heading)
                .build();

        trajSubmersiblePreloadDropToSpecimenPreload2 = drive.actionBuilder(submersibleSpecimenPreloadDrop)
                //.setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(pickupSpecimenPreload2, Math.toRadians(0))
                .build();

        trajSpecimenPreload2ToSubmersiblePreload2 = drive.actionBuilder(pickupSpecimenPreload2)
                //.setReversed(false)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenPreload2, Math.toRadians(-180))
                .build();

        trajSubmersiblePreload2ToSubmersiblePreload2Drop = drive.actionBuilder(submersibleSpecimenPreload2)
                .strafeToLinearHeading(submersibleSpecimenPreload2Drop.position, submersibleSpecimenPreload2Drop.heading)
                .build();

        trajSubmersiblePreload2DropToPickupSpecimenOne = drive.actionBuilder(submersibleSpecimenPreload2Drop)
                .setTangent(0)
                .splineToLinearHeading(preColorSampleNear, Math.toRadians(90))
                .strafeToLinearHeading(colorSampleNear.position, colorSampleNear.heading)
                .strafeToLinearHeading(colorSampleNearDrop.position, colorSampleNearDrop.heading)
                .strafeToLinearHeading(preColorSampleMiddle.position, preColorSampleMiddle.heading)
                .strafeToLinearHeading(colorSampleMiddle.position, colorSampleMiddle.heading)
                .strafeToLinearHeading(colorSampleMiddleDrop.position, colorSampleMiddleDrop.heading)
                .strafeToLinearHeading(preColorSampleFar.position, preColorSampleFar.heading)
                .strafeToLinearHeading(colorSampleFar.position, colorSampleFar.heading)
                .strafeToLinearHeading(colorSampleFarDrop.position, colorSampleFarDrop.heading)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();


        /*trajSubmersiblePreload2DropToPickupSpecimenOne = drive.actionBuilder(submersibleSpecimenPreload2Drop)
                .setTangent(0)
                .splineToLinearHeading(preColorSampleNear, Math.toRadians(90))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(colorSampleNear,Math.toRadians(90))
                .strafeToLinearHeading(colorSampleNearDrop.position, colorSampleNearDrop.heading)
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(colorSampleMiddle,Math.toRadians(90))
                .strafeToLinearHeading(colorSampleMiddleDrop.position, colorSampleMiddleDrop.heading)
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(colorSampleFar,Math.toRadians(90))
                .strafeToLinearHeading(colorSampleFarDrop.position, colorSampleFarDrop.heading)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

        /*trajSubmersiblePreload2DropToColorSampleNear = drive.actionBuilder(submersibleSpecimenPreload2Drop)
                .setTangent(0)
                .splineToLinearHeading(preColorSampleNear, Math.toRadians(90))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(colorSampleNear,)
                .build();

        trajColorSampleNearToObservationDrop = drive.actionBuilder(colorSampleNear)
                .setTangent(Math.toRadians(-180))
                .build();

        trajObservationDropToColorSampleMiddle = drive.actionBuilder(observationDrop)
                .turnTo(colorSampleMiddle.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajColorSampleMiddleToObservationDrop = drive.actionBuilder(colorSampleMiddle)
                .turnTo(observationDrop.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajObservationDropToColorSampleFar = drive.actionBuilder(observationDrop)
                .turnTo(colorSampleFar.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajColorSampleFarToObservationDrop = drive.actionBuilder(colorSampleFar)
                .turnTo(observationDrop.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajObservationDropToPickupSpecimenOne = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

         */

        trajPickupSpecimenOneToSubmersibleOne = drive.actionBuilder(pickupSpecimenOne)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenOne, Math.toRadians(-180))
                .build();

        trajSubmersibleOneToSubmersibleOneDrop = drive.actionBuilder(submersibleSpecimenOne)
                .strafeToLinearHeading(submersibleSpecimenOneDrop.position, submersibleSpecimenOneDrop.heading)
                .build();

        trajSubmersibleOneDropToPickupSpecimenTwo = drive.actionBuilder(submersibleSpecimenOneDrop)
                //.setReversed(false)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(pickupSpecimenTwo, Math.toRadians(0))
                .build();

        trajPickupSpecimenTwoToSubmersibleTwo = drive.actionBuilder(pickupSpecimenTwo)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenTwo, Math.toRadians(-180))
                .build();

        trajSubmersibleTwoToSubmersibleTwoDrop = drive.actionBuilder(submersibleSpecimenTwo)
                .strafeToLinearHeading(submersibleSpecimenTwoDrop.position, submersibleSpecimenTwo.heading)
                .build();

        trajSubmersibleTwoDropToPickupSpecimenThree = drive.actionBuilder(submersibleSpecimenTwoDrop)
                //.setReversed(false)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(pickupSpecimenThree, Math.toRadians(0))//81.5
                .build();

        trajPickupSpecimenThreeToSubmersibleThree = drive.actionBuilder(pickupSpecimenThree)
                //.setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersibleSpecimenThree, Math.toRadians(-180))
                .build();

        trajSubmersibleThreeToSubmersibleThreeDrop = drive.actionBuilder(submersibleSpecimenThree)
                .strafeToLinearHeading(submersibleSpecimenThreeDrop.position, submersibleSpecimenThreeDrop.heading)
                .build();

        trajSubmersibleThreeDropToObservationPark = drive.actionBuilder(submersibleSpecimenThreeDrop)
                //.setReversed(false)
                .splineToLinearHeading(observationPark, Math.toRadians(57))
                .build();
    }

    public void runAutonomousMode() {
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(intialWaitTime),
                        //*** Drop Prelaod specimen
                        intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.SPECIMEN_PICKUP),
                        new ParallelAction(
                                trajInitToSubmersiblePreload,
                                intakeOuttakeController.moveOuttakeToHighChamberAction()
                        ),
                        //new SleepAction(0.3),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        trajSubmersiblePreloadToSubmersiblePreloadDrop,
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //*** Drop Specimen Preload to Pick and drop Specimen Preload 2
                        new ParallelAction(
                                trajSubmersiblePreloadDropToSpecimenPreload2,
                                intakeOuttakeController.moveOuttakeToSpecimenPickUpAction()
                        ),
                        intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                        trajSpecimenPreload2ToSubmersiblePreload2,
                        new SleepAction(0.1),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        trajSubmersiblePreload2ToSubmersiblePreload2Drop,
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        trajSubmersiblePreload2DropToPickupSpecimenOne,
                        /*
                        //*** Drop Specimen Preload 2 to Move Color Sample Near to Observation Zone
                        trajSubmersiblePreload2DropToColorSampleNear,
                        intakeOuttakeController.moveOuttakeArmOnlyToAction(Outtake.ARM_STATE.INIT),
                        intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.4, 30),
                        new SleepAction(0.3),
                        intakeOuttakeController.pickupSequenceAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajColorSampleNearToObservationDrop,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.6, 90)
                        ),
                        new SleepAction(0.2),
                        intakeOuttakeController.openIntakeGripAction(),

                        //*** Move Color Sample Middle to Observation Zone
                        trajObservationDropToColorSampleMiddle,
                        intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.6, 30),
                        new SleepAction(0.2),
                        intakeOuttakeController.pickupSequenceAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajColorSampleMiddleToObservationDrop,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.75, 90)
                        ),
                        new SleepAction(0.2),
                        intakeOuttakeController.openIntakeGripAction(),

                        //*** Move Color Sample Far to Observation Zone
                        pickFarSampleAndDropToObservationDropAction(),

                        //*** Ger Ready for Specimen Pickup
                        intakeOuttakeController.moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED),
                        intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.SPECIMEN_PICKUP),
                        new SleepAction(0.3),
                        intakeOuttakeController.moveOuttakeToSpecimenPickUpAction(),

                        //Observation Drop to Pick and drop Specimen 1
                        trajObservationDropToPickupSpecimenOne,

                         */
                        intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                        trajPickupSpecimenOneToSubmersibleOne,
                        new SleepAction(0.2),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        trajSubmersibleOneToSubmersibleOneDrop,
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //Drop Specimen 1 to Pick and drop Specimen 2
                        new ParallelAction(
                                trajSubmersibleOneDropToPickupSpecimenTwo,
                                intakeOuttakeController.moveOuttakeToSpecimenPickUpAction()
                        ),
                        intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                        trajPickupSpecimenTwoToSubmersibleTwo,
                        new SleepAction(0.2),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        trajSubmersibleTwoToSubmersibleTwoDrop,
                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),

                        //Drop Specimen 2 to Pick and drop Specimen 3
                        dropSpecimenThreeToPickAndDropSpecimenThreeAction(),

                        //Submersible Three to Park
                        new ParallelAction(
                                trajSubmersibleThreeDropToObservationPark,
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1,0)
                        ),
                        new SleepAction(0.1)
                )
        );
    }

    /*public Action pickFarSampleAndDropToObservationDropAction(){
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (autoOption == AUTO_OPTION.FIVE_SPECIMEN_AUTO) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    //*** Move Color Sample Far to Observation Zone
                                    trajObservationDropToColorSampleFar,
                                    intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1.0, 30),
                                    new SleepAction(0.2),
                                    intakeOuttakeController.pickupSequenceAction(),
                                    new SleepAction(0.1),
                                    new ParallelAction(
                                            trajColorSampleFarToObservationDrop,
                                            intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(0.65, 90)
                                    ),
                                    new SleepAction(0.2),
                                    intakeOuttakeController.openIntakeGripAction()
                            )
                    );

                }
                return false;
            }
        };
    }*/

    public Action dropSpecimenThreeToPickAndDropSpecimenThreeAction(){
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (autoOption == AUTO_OPTION.FIVE_SPECIMEN_AUTO) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            trajSubmersibleTwoDropToPickupSpecimenThree,
                                            intakeOuttakeController.moveOuttakeToSpecimenPickUpAction()
                                    ),
                                    intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                                    trajPickupSpecimenThreeToSubmersibleThree,
                                    new SleepAction(0.2),
                                    intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                                    trajSubmersibleThreeToSubmersibleThreeDrop,
                                    intakeOuttakeController.dropSamplefromOuttakeOnlyAction()
                            )
                    );

                }
                return false;
            }
        };
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
        intakeArm.moveArm(IntakeArm.ARM_STATE.SPECIMEN_PICKUP);
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

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPositionAndPreload() {
        //telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
            telemetry.addData("    Red Alliance - Left Samples    ", "(B / O-Circle)");
            telemetry.addData("    Blue Alliance - Left Samples", "(Y / Δ-Triangle)");
            if (gamepadController.gp1GetCirclePress()) {
                GameField.startPosition = GameField.START_POSITION.LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if (gamepadController.gp1GetTrianglePress()) {
                GameField.startPosition = GameField.START_POSITION.LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }

            telemetry.update();
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Alliance", GameField.allianceColor);
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select 4 Sample or 5 Sample Mode:");
            telemetry.addData("    4 Specimen     ", "(A / X-Cross)");
            telemetry.addData("    5 Specimen     ", "(X / ▢-Square)");

            if (gamepadController.gp1GetCrossPress()) {
                autoOption = AUTO_OPTION.FOUR_SPECIMEN_AUTO;
                break;
            }

            if (gamepadController.gp1GetSquarePress()) {
                autoOption = AUTO_OPTION.FIVE_SPECIMEN_AUTO;
                break;
            }

            telemetry.update();
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Alliance", GameField.allianceColor);
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Preload Sample on Outtake and Click Right Bumper to Close Grip");

            if (gamepadController.gp1GetRightBumperPress()) {
                outtake.closeGrip();
                break;
            }

            telemetry.update();
        }
        /*telemetry.addData("    A Cross Control     ", "(A / X)");
        telemetry.addData("    X Square ", "(X / ▢)");*/
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
        }
        telemetry.update();
    }
}
