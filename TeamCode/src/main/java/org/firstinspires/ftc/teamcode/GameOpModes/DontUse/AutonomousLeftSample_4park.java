
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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Autonomous(name = "Hazmat Auto LEFT 4+Park", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousLeftSample_4park extends LinearOpMode {

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
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;
        GameField.startPosition = GameField.START_POSITION.LEFT;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Selected Starting Position", GameField.startPosition);

        //Build trajectories
        buildAutonoumousMode();

        //lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("LEFT SAMPLE AUTO");
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
    Pose2d firstBucket = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d yellowSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d nearBucket = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d middleBucket = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d farBucket = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d yellowSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d yellowSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersiblePick = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersiblePrePark = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersiblePark = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToFirstBucket,
            trajFirstBucketToYellowSampleNear, trajYellowSampleNearToBucket,
            trajBucketToYellowSampleMiddle, trajYellowSampleMiddleToBucket,
            trajBucketToYellowSampleFar, trajYellowSampleFarToBucket,
            trajBucketToSubmersiblePick, trajSubmersiblePickToBucket,
            trajBucketToSubmersiblePark;

    public void buildAutonoumousMode() {
        //If initial action is moves too much in
        drive = new MecanumDrive(hardwareMap, initPose);
        firstBucket = new Pose2d(15.28, 34.16, Math.toRadians(-23.5));//14.5, 36
        yellowSampleNear = new Pose2d(15.28 + 5.0*Math.cos(Math.toRadians(90-23.5)),
                34.16 - Math.cos(Math.toRadians(90-23.5)), Math.toRadians(-23.5));
        //yellowSampleNear = new Pose2d(17.24, 29.56, Math.toRadians(-23.5));
        nearBucket = firstBucket;
        // nearBucket = new Pose2d(14.5, 36, Math.toRadians(-23.5));
        yellowSampleMiddle = new Pose2d(12.8, 20.5, Math.toRadians(-8));;
        middleBucket = new Pose2d(10, 21, Math.toRadians(-11));
        yellowSampleFar = new Pose2d(14.3, 20.7, Math.toRadians(13.7));
        farBucket = new Pose2d(10.5, 20.1, Math.toRadians(-23));
        submersiblePick = new Pose2d(53, -16, Math.toRadians(-90));
        submersiblePrePark = new Pose2d(47, 11, Math.toRadians(60));
        submersiblePark = new Pose2d(50, -18.5, Math.toRadians(-90));

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToFirstBucket = drive.actionBuilder(initPose)
                //.strafeToLinearHeading(preBucket.position, preBucket.heading)
                //.strafeToLinearHeading(firstBucket.position, firstBucket.heading)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(firstBucket, Math.toRadians(90))
                .build();

        //move to yellow sample one
        trajFirstBucketToYellowSampleNear = drive.actionBuilder(firstBucket)
                .strafeToLinearHeading(yellowSampleNear.position, yellowSampleNear.heading)
                .build();

        trajYellowSampleNearToBucket = drive.actionBuilder(yellowSampleNear)
                .strafeToLinearHeading(nearBucket.position, nearBucket.heading,
                        new TranslationalVelConstraint(24.0), new ProfileAccelConstraint(-15.0, 15.0))
                .build();

        trajBucketToYellowSampleMiddle = drive.actionBuilder(nearBucket)
                .strafeToLinearHeading(yellowSampleMiddle.position, yellowSampleMiddle.heading)
                .build();

        trajYellowSampleMiddleToBucket = drive.actionBuilder(yellowSampleMiddle)
                .strafeToLinearHeading(middleBucket.position, middleBucket.heading,
                        new TranslationalVelConstraint(27.0), new ProfileAccelConstraint(-18.0, 18.0))
                .build();

        trajBucketToYellowSampleFar = drive.actionBuilder(middleBucket)
                .strafeToLinearHeading(yellowSampleFar.position, yellowSampleFar.heading,
                        new TranslationalVelConstraint(25.0), new ProfileAccelConstraint(-18.0, 18.0))
                .build();

        trajYellowSampleFarToBucket = drive.actionBuilder(yellowSampleFar)
                .strafeToLinearHeading(farBucket.position, farBucket.heading,
                        new TranslationalVelConstraint(27.0), new ProfileAccelConstraint(-18.0, 18.0))
                .build();

        trajBucketToSubmersiblePick = drive.actionBuilder(farBucket)
                .setTangent(0)
                .splineToLinearHeading(submersiblePick, Math.toRadians(90))
                .build();

        trajSubmersiblePickToBucket = drive.actionBuilder(submersiblePick)
                .setReversed(true)
                .setTangent(0)
                .splineToLinearHeading(farBucket, Math.toRadians(90))
                .build();

        trajBucketToSubmersiblePark = drive.actionBuilder(farBucket)
                .setTangent(0)
                //.splineToLinearHeading(submersiblePrePark, Math.toRadians(-15))
                //.setTangent(Math.toRadians(-90))
                .splineToLinearHeading(submersiblePark, Math.toRadians(-110))
                .build();

    }


    public void runAutonomousMode() {
        Actions.runBlocking(
                new ParallelAction(
                    printDebugMessagesAction(),
                    new SequentialAction(
                        new SleepAction(intialWaitTime),
                        //Init to First Bucket
                        trajInitToFirstBucket,
                        new SleepAction(1),
                        new ParallelAction(
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1.0, 20),
                                new SequentialAction(
                                        intakeOuttakeController.moveOuttakeHighBucketAction1(),
                                        intakeOuttakeController.dropSamplefromOuttakeAndMoveArmToPreTransferAction1(),
                                        intakeOuttakeController.moveOuttakeSlidesToTransferAction1()
                                )
                        ),
                        //First Bucket to Sample Near
                        trajFirstBucketToYellowSampleNear,
                        new SleepAction(0.13),
                        intakeOuttakeController.pickupSequenceAction1(),
                        //Sample Near to Bucket
                        new ParallelAction(
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction1(),
                                trajYellowSampleNearToBucket
                        ),
                        new ParallelAction(
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1.0, 0),
                                new SequentialAction(
                                        intakeOuttakeController.moveOuttakeHighBucketAction1(),
                                        intakeOuttakeController.dropSamplefromOuttakeAndMoveArmToPreTransferAction1(),
                                        intakeOuttakeController.moveOuttakeSlidesToTransferAction1()
                                )
                        )/*,

                        //Bucket to Sample Middle
                        trajBucketToYellowSampleMiddle,
                        new SleepAction(0.13),
                        intakeOuttakeController.pickupSequenceAction1(),
                        //Sample Middle to Bucket
                        new ParallelAction(
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction1(),
                                trajYellowSampleMiddleToBucket
                        ),
                        new ParallelAction(
                                intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1.0, -10),
                                new SequentialAction(
                                        intakeOuttakeController.moveOuttakeHighBucketAction1(),
                                        intakeOuttakeController.dropSamplefromOuttakeAndMoveArmToPreTransferAction1(),
                                        intakeOuttakeController.moveOuttakeSlidesToTransferAction1()
                                )
                        ),
                        //Bucket to Sample Far
                        trajBucketToYellowSampleFar,
                        new SleepAction(0.13),
                        intakeOuttakeController.pickupSequenceAction1(),
                        //Sample Far to Bucket
                        new ParallelAction(
                                intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransferAction1(),
                                trajYellowSampleFarToBucket
                        ),
                        intakeOuttakeController.moveOuttakeHighBucketAction1(),
                        intakeOuttakeController.dropSamplefromOuttakeAndMoveArmToPreTransferAction1(),
                        intakeOuttakeController.moveOuttakeSlidesToTransferAction1()


                        //Bucket to Submersible Park
                        new ParallelAction(
                                trajBucketToSubmersiblePark,
                                intakeOuttakeController.setToAutoEndStateSubmerssibleParkAction()
                        ),
                        new SleepAction(1)


                        */
                    )
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

        /*vision = new Vision(hardwareMap, telemetry);
        telemetry.addLine("Vision Initialized");
        telemetry.update();
         */

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
    public void printDebugMessages(){
        //telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addLine("Robot ready to start");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat TeleOpMode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            telemetry.addData("PoseEstimateString :", toStringPose2d(drive.pose));

            driveTrain.printDebugMessages();
            intakeArm.printDebugMessages();
            intakeSlides.printDebugMessages();
            outtake.printDebugMessages();
            //lights.printDebugMessages();
        }
        telemetry.update();
    }

    public Action printDebugMessagesAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                printDebugMessages();
                return true;
            }
        };
    }


}