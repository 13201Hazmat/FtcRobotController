
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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
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
import org.firstinspires.ftc.teamcode.SubSystems.VisionLimeLight;

/**
 * Hazmat Autonomous
 */

@Autonomous(name = "Hazmat Auto RIGHT 5 Specimen", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutonomousRight5Specimen extends LinearOpMode {

    public GamepadController gamepadController;
    public IntakeOuttakeController intakeOuttakeController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public VisionLimeLight vision;
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
    Pose2d colorSampleNear = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleMiddle = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleFar = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationDrop = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenPreload2 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenPreload2 = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d pickupSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersibleSpecimenThree = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));

    double intialWaitTime = 0;

    //List all Trajectories
    Action trajInitToSubmersiblePreload,
            trajSubmersiblePreloadToColorSampleFar, trajColorSampleFarToObservationDrop,
            trajObservationDropToColorSampleMiddle, trajColorSampleMiddleToObservationDrop,
            trajObservationDropToColorSampleNear, trajColorSampleNearToObservationDrop,
            trajObservationDropToPickupSpecimenOne,
            trajPickupSpecimenOneToSubmersibleOne,  trajSubmersibleOneToPickupSpecimenTwo,
            trajPickupSpecimenTwoToSubmersibleTwo,  trajSubmersibleTwoToPickupPreload2,
            trajPickupPreload2ToSubmersiblePreload2, trajSubmersiblePreload2ToPickupSpecimenThree,
            trajPickupSpecimenThreeToSubmersibleThree, trajSubmersibleThreeToObservationPark, trajSubmersiblePreload2ToObservationPark;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        drive = new MecanumDrive(hardwareMap, initPose);
        submersibleSpecimenPreload = new Pose2d(-29.75, -1.2, Math.toRadians(0));
        colorSampleNear = new Pose2d(-15.5, 37.5, Math.toRadians(167));//-16.5
        colorSampleMiddle = new Pose2d(-19, 37.5, Math.toRadians(145));//-20. -146.6
        colorSampleFar = new Pose2d(-18.5, 38.75, Math.toRadians(128));//19.5
        observationDrop = new Pose2d(-17, 36.7, Math.toRadians(30));
        pickupSpecimenOne = new Pose2d(-1.1, 42, Math.toRadians(3));//-0.6
        submersibleSpecimenOne = new Pose2d(-30.25, -14, Math.toRadians(0));//-29.5
        pickupSpecimenTwo = new Pose2d(-1, 32, Math.toRadians(3));//46,0
        submersibleSpecimenTwo = new Pose2d(-29, -15, Math.toRadians(0));//-27.5
        pickupSpecimenPreload2 = new Pose2d(-3, 32, Math.toRadians(3)); //-2
        submersibleSpecimenPreload2 =  new Pose2d(-29, -16, Math.toRadians(0));//-28
        pickupSpecimenThree = new Pose2d(-9, 32, Math.toRadians(3));//-6.5
        submersibleSpecimenThree = new Pose2d(-29.2, -17, Math.toRadians(0));//-25
        observationPark = new Pose2d(-7.3, 7.4, Math.toRadians(70));

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToSubmersiblePreload = drive.actionBuilder(initPose)
                .strafeToLinearHeading(submersibleSpecimenPreload.position, submersibleSpecimenPreload.heading)
                .build();

        trajSubmersiblePreloadToColorSampleFar = drive.actionBuilder(submersibleSpecimenPreload)
                .setTangent(Math.toRadians(45)) //0
                .splineToLinearHeading(colorSampleFar, Math.toRadians(90))
                .build();

        trajColorSampleFarToObservationDrop = drive.actionBuilder(colorSampleFar)
                .turnTo(observationDrop.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajObservationDropToColorSampleMiddle = drive.actionBuilder(observationDrop)
                .turnTo(colorSampleMiddle.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajColorSampleMiddleToObservationDrop = drive.actionBuilder(colorSampleMiddle)
                .turnTo(observationDrop.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajObservationDropToColorSampleNear = drive.actionBuilder(observationDrop)
                .turnTo(colorSampleNear.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajColorSampleNearToObservationDrop = drive.actionBuilder(colorSampleNear)
                .turnTo(observationDrop.heading, new TurnConstraints(2*Math.PI, -2*Math.PI, 2*Math.PI))
                .build();

        trajObservationDropToPickupSpecimenOne = drive.actionBuilder(observationDrop)
                .strafeToLinearHeading(pickupSpecimenOne.position, pickupSpecimenOne.heading)
                .build();

        trajPickupSpecimenOneToSubmersibleOne = drive.actionBuilder(pickupSpecimenOne)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(submersibleSpecimenOne, Math.toRadians(-180),
                        new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajSubmersibleOneToPickupSpecimenTwo = drive.actionBuilder(submersibleSpecimenOne)
                .setTangent(Math.toRadians(3))
                .splineToSplineHeading(pickupSpecimenTwo, Math.toRadians(-15),
                        new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajPickupSpecimenTwoToSubmersibleTwo = drive.actionBuilder(pickupSpecimenTwo)
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(submersibleSpecimenTwo, Math.toRadians(-180),
                        new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajSubmersibleTwoToPickupPreload2 = drive.actionBuilder(submersibleSpecimenTwo)
                .setTangent(Math.toRadians(3))
                .splineToSplineHeading(pickupSpecimenPreload2, Math.toRadians(-15),
                        new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajPickupPreload2ToSubmersiblePreload2 = drive.actionBuilder(pickupSpecimenPreload2)
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(submersibleSpecimenPreload2, Math.toRadians(-180),
                        new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajSubmersiblePreload2ToPickupSpecimenThree = drive.actionBuilder(submersibleSpecimenPreload2)
                .setTangent(Math.toRadians(3))
                .splineToSplineHeading(pickupSpecimenThree, Math.toRadians(-15),
                        new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajPickupSpecimenThreeToSubmersibleThree = drive.actionBuilder(pickupSpecimenThree)
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(submersibleSpecimenThree, Math.toRadians(-180),
                        new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajSubmersibleThreeToObservationPark = drive.actionBuilder(submersibleSpecimenThree)
                .splineToSplineHeading(observationPark, Math.toRadians(57),
                        new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-65.0, 85.0))
                .build();

        trajSubmersiblePreload2ToObservationPark = drive.actionBuilder(submersibleSpecimenPreload2)
                .splineToSplineHeading(observationPark, Math.toRadians(57),
                        new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-65.0, 85.0))
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
                        new SleepAction(0.1),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                trajSubmersiblePreloadToColorSampleFar,
                                new SequentialAction(
                                        new SleepAction(0.2),//TODO 0.3
                                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                        new SleepAction(0.1),
                                        intakeOuttakeController.moveOuttakeArmToAction(Outtake.ARM_STATE.INIT),
                                        intakeOuttakeController.moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.SPECIMEN_PICKUP)
                                ),
                                intakeOuttakeController.openIntakeGripAction(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        intakeOuttakeController.extendIntakeArmSwivelValueToPrePickupByExtensionFactorAction(1.0, 0.324)
                                )
                        ),
                        //*** Drop Specimen Preload to Move Color Sample Far to Observation Zone
                        //TODO new SleepAction(0.2),
                        intakeOuttakeController.pickupSequenceAction(),
                        //TODO new SleepAction(0.1),
                        new ParallelAction(
                                trajColorSampleFarToObservationDrop,
                                intakeOuttakeController.extendIntakeArmSwivelValueToPrePickupByExtensionFactorAction(0.65, 0.324)
                        ),
                        //new SleepAction(0.1),
                        intakeOuttakeController.openIntakeGripAction(),
                        new SleepAction(0.1),

                        //*** Move Color Sample Middle to Observation Zone
                        new ParallelAction(
                                intakeOuttakeController.extendIntakeArmSwivelValueToPrePickupByExtensionFactorAction(0.65, 0.36),
                                trajObservationDropToColorSampleMiddle
                        ),
                        //TODO new SleepAction(0.1),
                        intakeOuttakeController.pickupSequenceAction(),
                        //new SleepAction(0.1),
                        new ParallelAction(
                                intakeOuttakeController.extendIntakeArmSwivelValueToPrePickupByExtensionFactorAction(0.65, 0.36),
                                trajColorSampleMiddleToObservationDrop
                        ),
                        //new SleepAction(0.1),
                        intakeOuttakeController.openIntakeGripAction(),
                        new SleepAction(0.1),

                        //*** Move Color Sample Near to Observation Zone
                        pickNearSampleAndDropToObservationDropAction(),

                        //*** Ger Ready for Specimen Pickup
                        intakeOuttakeController.moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED),
                        intakeOuttakeController.moveIntakeArmToAction(IntakeArm.ARM_STATE.SPECIMEN_PICKUP),
                        new SleepAction(0.3),
                        intakeOuttakeController.moveOuttakeToSpecimenPickUpAction(),

                        //Observation Drop to Pick and drop Specimen 1
                        trajObservationDropToPickupSpecimenOne,
                        new SleepAction(0.2), //TODO 0.3
                        intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                        trajPickupSpecimenOneToSubmersibleOne,
                        new SleepAction(0.1),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajSubmersibleOneToPickupSpecimenTwo,
                                new SequentialAction(
                                        new SleepAction(0.2),//TODO 0.3
                                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                        new SleepAction(0.1),
                                        intakeOuttakeController.moveOuttakeToSpecimenPickUpAction()
                                )
                        ),


                        //Drop Specimen 1 to Pick and drop Specimen 2
                        new SleepAction(0.2), //TODO 0.3
                        intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                        trajPickupSpecimenTwoToSubmersibleTwo,
                        new SleepAction(0.1),
                        intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                trajSubmersibleTwoToPickupPreload2,
                                new SequentialAction(
                                        new SleepAction(0.2),//TODO 0.3
                                        intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                        new SleepAction(0.1),
                                        intakeOuttakeController.moveOuttakeToSpecimenPickUpAction()
                                )
                        ),

                        //Drop Specimen 2 to Pick and drop Specimen Preload, Specimen3 and Park
                        pickPreload2ToDropPreload2PickAndDropSpecimenThreeAndParkAction()


                )
        );
    }

    public Action pickNearSampleAndDropToObservationDropAction(){
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
                                    new ParallelAction(
                                            intakeOuttakeController.extendIntakeArmSwivelValueToPrePickupByExtensionFactorAction(0.35, 0.41),
                                            trajObservationDropToColorSampleNear
                                    ),
                                    //new SleepAction(0.25),
                                    intakeOuttakeController.pickupSequenceAction(),
                                    //new SleepAction(0.1),
                                    new ParallelAction(
                                            trajColorSampleNearToObservationDrop,
                                            intakeOuttakeController.extendIntakeArmSwivelValueToPrePickupByExtensionFactorAction(0.65, 0.41)
                                    ),
                                    //new SleepAction(0.1),
                                    intakeOuttakeController.openIntakeGripAction()
                                    //new SleepAction(0.1)

                            )
                    );

                }
                return false;
            }
        };
    }

    public Action pickPreload2ToDropPreload2PickAndDropSpecimenThreeAndParkAction(){
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (autoOption == AUTO_OPTION.FIVE_SPECIMEN_AUTO) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    //*** Drop Specimen Preload to Pick and drop Specimen Preload 2
                                    new SleepAction(0.2), //TODO 0.3
                                    intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                                    trajPickupPreload2ToSubmersiblePreload2,
                                    new SleepAction(0.1),
                                    intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                                    new SleepAction(0.1),
                                    new ParallelAction(
                                            trajSubmersiblePreload2ToPickupSpecimenThree,
                                            new SequentialAction(
                                                    new SleepAction(0.2),//TODO 0.3
                                                    intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                                    new SleepAction(0.1),
                                                    intakeOuttakeController.moveOuttakeToSpecimenPickUpAction()
                                            )
                                    ),
                                    intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                                    trajSubmersiblePreload2ToPickupSpecimenThree,
                                    new SleepAction(0.1),
                                    intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                                    new SleepAction(0.1),
                                    new ParallelAction(
                                            trajPickupSpecimenThreeToSubmersibleThree,
                                            new SequentialAction(
                                                    new SleepAction(0.2),//TODO 0.3
                                                    intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                                    new SleepAction(0.1),
                                                    intakeOuttakeController.moveOuttakeArmOnlyToAction(Outtake.ARM_STATE.INIT)
                                            )
                                    ),
                                    intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1,0),
                                    new SleepAction(0.1)
                            )
                    );
                } else {
                    Actions.runBlocking(
                            new SequentialAction(
                                    //*** Drop Specimen Preload to Pick and drop Specimen Preload 2
                                    //0.2
                                    new SleepAction(0.7),
                                    intakeOuttakeController.pickupSpecimenAndMoveOuttakeToHighChamberAction(),
                                    trajPickupPreload2ToSubmersiblePreload2,
                                    new SleepAction(0.1),
                                    intakeOuttakeController.moveOuttakeToHighChamberLatchAction(),
                                    new SleepAction(0.1),
                                    new ParallelAction(
                                            trajSubmersiblePreload2ToObservationPark,
                                            new SequentialAction(
                                                    new SleepAction(0.3),
                                                    intakeOuttakeController.dropSamplefromOuttakeOnlyAction(),
                                                    new SleepAction(0.1),
                                                    intakeOuttakeController.moveOuttakeArmOnlyToAction(Outtake.ARM_STATE.INIT)
                                            )
                                    ),
                                    intakeOuttakeController.extendIntakeArmSwivelToPrePickupByExtensionFactorAction(1,0),
                                    new SleepAction(0.5)
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

        vision = new VisionLimeLight(hardwareMap, telemetry);
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
        /*while (!isStopRequested()) {
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
        }*/

        GameField.startPosition = GameField.START_POSITION.LEFT;
        GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
        autoOption = AUTO_OPTION.FIVE_SPECIMEN_AUTO;

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            //telemetry.addData("Selected Alliance", GameField.allianceColor);
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
