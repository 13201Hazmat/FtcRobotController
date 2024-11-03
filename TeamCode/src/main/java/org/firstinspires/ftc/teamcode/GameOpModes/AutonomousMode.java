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

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadDriveTrainController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.SpecimenHandler;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "Hazmat Autonomous Mode", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")

public class AutonomousMode extends LinearOpMode {

    public GamepadController gamepadController;
    public GamepadDriveTrainController gamepadDriveTrainController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public SpecimenHandler specimenHandler;
    public Climber climber;
    public Lights lights;

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

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

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
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);

            runAutonomousMode();
        }
    }   // end runOpMode()

    //List All Poses
    //LEFT
    Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0)); // Starting Pose
    Pose2d submersibleSpecimen = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d netZone = new Pose2d(8, 0, Math.toRadians(0));
    Pose2d yellowSampleOne = new Pose2d(0, 0, Math.toRadians(0));

    Pose2d yellowSampleTwo = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d submersiblePark = new Pose2d(0, 0, Math.toRadians(0));

    //RIGHT
    Pose2d observationZone = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d specimenPickup = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d colorSampleOne = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d observationPark = new Pose2d(0, 0, Math.toRadians(0));

    double waitSecondsBeforeDrop = 0;
    MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

    //List all Trajectories
    Action trajInitToDropPurplePixel, trajInitToDropPurplePixelLeft, trajInitToDropPurplePixelMiddle, trajInitToDropPurplePixelRight;


    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        switch (GameField.startPosition) {
            case LEFT:

                Pose2d submersibleSpecimen = new Pose2d(18, -23, Math.toRadians(0));
                Pose2d netZone = new Pose2d(8, 15, Math.toRadians(120));
                Pose2d yellowSampleOne = new Pose2d(21, 12, Math.toRadians(0));

                Pose2d yellowSampleTwo = new Pose2d(21, 30, Math.toRadians(0));
                Pose2d submersiblePark = new Pose2d(39, -33, Math.toRadians(0));
                break;

            case RIGHT:
                Pose2d observationZone = new Pose2d(-8.96, -10.53, Math.toRadians(-160));
                Pose2d specimenPickup = new Pose2d(-13.00, 14.96, Math.toRadians(-180));
                Pose2d colorSampleOne = new Pose2d(0, 9.97, Math.toRadians(-1.28));
                Pose2d observationPark = new Pose2d(-25.71, 6.34, Math.toRadians(1.31));

                break;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        //Add trajectories..
        /*trajInitToDropPurplePixelLeft = drive.actionBuilder(initPose)
                .strafeTo(moveBeyondTrussPose.position)
                .splineToLinearHeading(dropPurplePixelPoseLeft, 0, new TranslationalVelConstraint(25.0),
                        new ProfileAccelConstraint(-20.0, 20.0))
                .build();
        trajInitToDropPurplePixelMiddle = drive.actionBuilder(initPose)
                .strafeTo(moveBeyondTrussPose.position)
                .splineToLinearHeading(dropPurplePixelPoseMiddle, 0, new TranslationalVelConstraint(24.0),
                        new ProfileAccelConstraint(-20.0, 20.0))
                .build();
        trajInitToDropPurplePixelRight = drive.actionBuilder(initPose)
                .strafeTo(moveBeyondTrussPose.position)
                .splineToLinearHeading(dropPurplePixelPoseRight, 0, new TranslationalVelConstraint(24.0),
                        new ProfileAccelConstraint(-20.0, 20.0))
                .build();


         */
    }

    public void runAutonomousMode() {
        if (GameField.startPosition == GameField.START_POSITION.LEFT) {
               /*     Actions.runBlocking(
                            new SequentialAction(
                                    intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                                    intakeController.dropLiftIntake(),
                                    trajInitToDropPurplePixel,
                                    intakeController.dropPurplePixelUsingIntakeAction(),
                                    new SleepAction(0.5), //ADD FOR SYNCHRONIZING TIME
                                    intakeController.intakeLiftUpAction(),

                                    new SleepAction(1.5),
                                    new SleepAction(afterPurplePixelWait),
                                    trajDropPurplePixelToAfterPurplePixel,
                                    trajAfterPurplePixelToDropYellowPixel,
                                    outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST_AUTO),
                                    new SleepAction(0.5),
                                    outtakeController.dropOnePixelAction(),
                                    new SleepAction(2),
                                    trajDropYellowPixelToAfterYellowPixel,
                                    outtakeController.moveOuttakeToEndStateAction(),
                                    new SleepAction(afterYellowPixelWait),
                                    //Go to Park
                                    new ParallelAction(
                                            trajAfterYellowPixelToPark,
                                            parkingArm.extendParkingArmAction()
                                    )
                            )
               */

        } else { //autoOption == RIGHT

            //Action Sequence
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
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
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
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addLine("Select Auto Options");
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
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public void initSubsystems(){

        telemetry.setAutoClear(false);

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

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadDriveTrainController = new GamepadDriveTrainController(gamepad1, driveTrain, this);
        telemetry.addLine("Gamepad DriveTrain Initialized");
        telemetry.update();

        gamepadController = new GamepadController(gamepad1, gamepad2, lights, intakeArm, intakeSlides,
                outtake, specimenHandler, climber, telemetry, this);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

    public String toStringPose2d(Pose2d pose){
        return String.format("(%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.log()));
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
            lights.printDebugMessages();
        }
        telemetry.update();
    }


}



