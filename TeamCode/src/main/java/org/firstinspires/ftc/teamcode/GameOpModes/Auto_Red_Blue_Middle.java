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
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.ParkingArm;
import org.firstinspires.ftc.teamcode.SubSystems.VisionOpenCV;
import org.firstinspires.ftc.teamcode.SubSystems.VisionSensor;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "Red_Blue_Middle", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")

public class Auto_Red_Blue_Middle extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Climber climber;
    public Launcher launcher;
    public ParkingArm parkingArm;
    public VisionSensor visionSensor;
    //public VisionTfod visionTfodFront;
    public VisionOpenCV visionOpenCV;
    public Lights lights;
    public OuttakeController outtakeController;
    public IntakeController intakeController;
    public TelemetryPacket telemetryPacket = new TelemetryPacket();

    public MecanumDrive drive;

    //Static Class for knowing system state

    public enum AUTO_OPTION{
        PRELOAD_AND_PARK,
    }
    public static AUTO_OPTION autoOption;

    public enum PARKING_OPTION{
        CORNER_WALL_RIGGING,
        FRONT_OF_WALL_RIGGING,
        FRONT_OF_BACKDROP
    }
    public static PARKING_OPTION parkingOption;

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

        // Initiate Camera on Init.
        //visionTfodFront.initTfod();
        visionOpenCV.initOpenCV();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addData("Selected After Drop Purple Pixel Wait", afterPurplePixelWait);
            telemetry.addData("Selected After Drop Yellow Pixel Wait", afterYellowPixelWait);
            telemetry.addData("Selected Parking", parkingOption);
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            //visionTfodFront.runTfodTensorFlow();
            visionOpenCV.runOpenCVObjectDetection();
            telemetry.addData("Vision identified SpikeMark Location", visionOpenCV.identifiedSpikeMarkLocation);
            telemetry.update();

            //Build parking trajectory based on last detected target by vision
            setTrajectoryBasedOnVision();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);

            runActionForRedRightBlueLeft();
        }
    }   // end runOpMode()

    Pose2d initPose= new Pose2d(0, 0, 0);
    Pose2d moveBeyondTrussPose = new Pose2d(6.5, 0, 0);

    Pose2d dropPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d afterPurplePixelPose1 = new Pose2d(0, 0, 0); //TODO: FIX BUG
    Pose2d afterPurplePixelPose2 = new Pose2d(0, 0, 0); //TODO: FIX BUG
    Pose2d afterPurplePixelPose3 = new Pose2d(0, 0, 0); //TODO: FIX BUG

    Pose2d dropYellowPixelPoseLeft= new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPoseMiddle= new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPoseRight= new Pose2d(0, 0, 0);

    Pose2d afterYellowPixelPose = new Pose2d(0, 0, 0); //TODO: FIX BUG

    Pose2d parkPoseCornerWallRigging = new Pose2d(0, 0, 0);
    Pose2d parkPoseFrontOfWallRigging = new Pose2d(0, 0, 0);
    Pose2d parkPoseFrontOfBackDrop = new Pose2d(0, 0, 0);
    Pose2d parkPose = new Pose2d(0, 0, 0);

    Action trajInitToDropPurplePixel, trajInitToDropPurplePixelLeft, trajInitToDropPurplePixelMiddle,trajInitToDropPurplePixelRight;
    Action trajDropPurplePixelToAfterPurplePixel, trajDropPurplePixelToAfterPurplePixelLeft,
            trajDropPurplePixelToAfterPurplePixelMiddle, trajDropPurplePixelToAfterPurplePixelRight;
    Action trajAfterPurplePixelToDropYellowPixel, trajAfterPurplePixelToDropYellowPixelLeft,trajAfterPurplePixelToDropYellowPixelMiddle, trajAfterPurplePixelToDropYellowPixelRight;

    Action trajDropYellowPixelToAfterYellowPixel, trajDropYellowPixelToAfterYellowPixelLeft, trajDropYellowPixelToAfterYellowPixelMiddle, trajDropYellowPixelToAfterYellowPixelRight;
    Action trajAfterYellowPixelToPark;
    double afterPurplePixelWait = 0, afterYellowPixelWait = 0;


    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        switch (GameField.startPosition) {
            case BLUE_MIDDLE:
                drive = new MecanumDrive(hardwareMap, initPose);
                dropPurplePixelPoseLeft = new Pose2d(30, 19, Math.toRadians(-90)); //x22.2, 3.3, 25
                dropYellowPixelPoseLeft = new Pose2d(20, 82, Math.toRadians(-90));//x16.7, y32.5

                dropPurplePixelPoseMiddle = new Pose2d(25, 0, Math.toRadians(0)); //x28, y-1.6, 10.7
                dropYellowPixelPoseMiddle = new Pose2d(27, 82, Math.toRadians(-90));//x25, y35

                dropPurplePixelPoseRight = new Pose2d(28, -3, Math.toRadians(-90)); //22.7,-6.7,-60//x24.5, y-9, -36.7
                dropYellowPixelPoseRight = new Pose2d(33, 82, Math.toRadians(-90)); //y=33.5, x35

                afterPurplePixelPose1 = new Pose2d(5, 21, Math.toRadians(-90));;
                afterPurplePixelPose2 = new Pose2d(5, 65, Math.toRadians(-90));;
                afterPurplePixelPose3 = new Pose2d(27, 70, Math.toRadians(-90));;


                afterYellowPixelPose = new Pose2d(5, 60, Math.toRadians(-90));

                parkPoseCornerWallRigging = new Pose2d(0, 92, Math.toRadians(-90)); //x2, x26
                parkPoseFrontOfWallRigging =  new Pose2d(2, 67, Math.toRadians(-90));;
                parkPoseFrontOfBackDrop = new Pose2d(24, 63, Math.toRadians(-90)); //x50, y30

                break;

            case RED_MIDDLE:
                drive = new MecanumDrive(hardwareMap, initPose);
                dropPurplePixelPoseLeft = new Pose2d(26.5, 4, Math.toRadians(90));//x20.5, y8,51
                dropYellowPixelPoseLeft = new Pose2d(35, -83, Math.toRadians(88)); //x=31.5,y=-32

                dropPurplePixelPoseMiddle = new Pose2d(26, -2, Math.toRadians(0));//x27.6, y0, 13
                dropYellowPixelPoseMiddle = new Pose2d(27, -83, Math.toRadians(90));//x27, y-32

                dropPurplePixelPoseRight = new Pose2d(27, -20, Math.toRadians(90));//x26.8, y-21.6
                dropYellowPixelPoseRight = new Pose2d(20, -83, Math.toRadians(90));//x18, y-32

                afterPurplePixelPose1 = new Pose2d(5, -21, Math.toRadians(90));;
                afterPurplePixelPose2 = new Pose2d(5, -65, Math.toRadians(90));
                afterPurplePixelPose3 = new Pose2d(27, -70, Math.toRadians(90));;

                afterYellowPixelPose = new Pose2d(5, -60, Math.toRadians(90));;

                parkPoseCornerWallRigging = new Pose2d(0, -92, Math.toRadians(90));
                parkPoseFrontOfWallRigging =  new Pose2d(4, -67, Math.toRadians(90));;
                parkPoseFrontOfBackDrop = new Pose2d(21, -63, Math.toRadians(90));
                break;
        }

        switch (parkingOption) {
            case CORNER_WALL_RIGGING:
                parkPose = parkPoseCornerWallRigging;
                break;
            case FRONT_OF_WALL_RIGGING:
                parkPose = parkPoseFrontOfWallRigging;
                break;
            case FRONT_OF_BACKDROP:
                parkPose = parkPoseFrontOfBackDrop;
                break;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToDropPurplePixelLeft = drive.actionBuilder(initPose)
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

        //FOR ONE_LOOP
        trajDropPurplePixelToAfterPurplePixelLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                .setReversed(true)
                .strafeToLinearHeading(afterPurplePixelPose1.position, afterPurplePixelPose1.heading)
                .build();
        trajDropPurplePixelToAfterPurplePixelMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                .setReversed(true)
                .strafeToLinearHeading(afterPurplePixelPose1.position, afterPurplePixelPose1.heading)
                .build();
        trajDropPurplePixelToAfterPurplePixelRight = drive.actionBuilder(dropPurplePixelPoseRight)
                .setReversed(true)
                .strafeToLinearHeading(afterPurplePixelPose1.position, afterPurplePixelPose1.heading)
                .build();

        trajAfterPurplePixelToDropYellowPixelLeft = drive.actionBuilder(afterPurplePixelPose1)
                .strafeToLinearHeading(afterPurplePixelPose2.position, afterPurplePixelPose2.heading)
                .strafeToLinearHeading(afterPurplePixelPose3.position, afterPurplePixelPose3.heading)
                .setReversed(true)
                .splineToLinearHeading(dropYellowPixelPoseLeft, 0)
                .build();
        trajAfterPurplePixelToDropYellowPixelMiddle = drive.actionBuilder(afterPurplePixelPose1)
                .strafeToLinearHeading(afterPurplePixelPose2.position, afterPurplePixelPose2.heading)
                .strafeToLinearHeading(afterPurplePixelPose3.position, afterPurplePixelPose3.heading)
                .setReversed(true)
                .splineToLinearHeading(dropYellowPixelPoseMiddle, 0)
                .build();
        trajAfterPurplePixelToDropYellowPixelRight = drive.actionBuilder(afterPurplePixelPose1)
                .strafeToLinearHeading(afterPurplePixelPose2.position, afterPurplePixelPose2.heading)
                .strafeToLinearHeading(afterPurplePixelPose3.position, afterPurplePixelPose3.heading)
                .setReversed(true)
                .splineToLinearHeading(dropYellowPixelPoseRight, 0)
                .build();

        trajDropYellowPixelToAfterYellowPixelLeft = drive.actionBuilder(dropYellowPixelPoseLeft)
                .strafeToLinearHeading(afterYellowPixelPose.position, afterYellowPixelPose.heading)
                .build();

        trajDropYellowPixelToAfterYellowPixelMiddle= drive.actionBuilder(dropYellowPixelPoseMiddle)
                .strafeToLinearHeading(afterYellowPixelPose.position, afterYellowPixelPose.heading)
                .build();

        trajDropYellowPixelToAfterYellowPixelRight = drive.actionBuilder(dropYellowPixelPoseRight)
                .strafeToLinearHeading(afterYellowPixelPose.position, afterYellowPixelPose.heading)
                .build();

        trajAfterYellowPixelToPark = drive.actionBuilder(afterYellowPixelPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();
    }

    public void setTrajectoryBasedOnVision() {
        switch (visionOpenCV.identifiedSpikeMarkLocation) {
            case LEFT:
                trajInitToDropPurplePixel = trajInitToDropPurplePixelLeft;
                trajDropPurplePixelToAfterPurplePixel = trajDropPurplePixelToAfterPurplePixelLeft;
                trajAfterPurplePixelToDropYellowPixel = trajAfterPurplePixelToDropYellowPixelLeft;
                trajDropYellowPixelToAfterYellowPixel = trajDropYellowPixelToAfterYellowPixelLeft;
                break;

            case MIDDLE:
                trajInitToDropPurplePixel = trajInitToDropPurplePixelMiddle;
                trajDropPurplePixelToAfterPurplePixel = trajDropPurplePixelToAfterPurplePixelMiddle;
                trajAfterPurplePixelToDropYellowPixel = trajAfterPurplePixelToDropYellowPixelMiddle;
                trajDropYellowPixelToAfterYellowPixel = trajDropYellowPixelToAfterYellowPixelMiddle;
                break;
            case RIGHT:
                trajInitToDropPurplePixel = trajInitToDropPurplePixelRight;
                trajDropPurplePixelToAfterPurplePixel = trajDropPurplePixelToAfterPurplePixelRight;
                trajAfterPurplePixelToDropYellowPixel = trajAfterPurplePixelToDropYellowPixelRight;
                trajDropYellowPixelToAfterYellowPixel = trajDropYellowPixelToAfterYellowPixelRight;
                break;
        }
    }

    public void runActionForRedRightBlueLeft() {
        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    new SequentialAction(
                            intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                            intakeController.dropLiftIntake(),
                            trajInitToDropPurplePixel,
                            intakeController.dropPurplePixelUsingIntakeAction(),
                            new SleepAction(0.5), //ADD FOR SYNCHRONIZING TIME
                            new ParallelAction(
                                    intakeController.intakeLiftUpAction(),
                                    trajDropPurplePixelToAfterPurplePixel
                            ),
                            new SleepAction(afterPurplePixelWait),
                            trajAfterPurplePixelToDropYellowPixel,
                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST_AUTO),
                            new SleepAction(0.5),
                            outtakeController.dropOnePixelAction(),
                            new SleepAction(2),
                            new ParallelAction(
                                    trajDropYellowPixelToAfterYellowPixel,
                                    outtakeController.moveOuttakeToEndStateAction()
                            ),
                            new SleepAction(afterYellowPixelWait),
                            //Go to Park
                            new ParallelAction(
                                    trajAfterYellowPixelToPark,
                                    parkingArm.extendParkingArmAction()
                            )
                    )
            );
        }


    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        //telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:","");
            telemetry.addData("    Red Middle  ", "(A / X)");
            telemetry.addData("    Blue Middle ", "(X / ▢)");
            if(gamepadController.gp1GetCrossPress()){
                GameField.startPosition = GameField.START_POSITION.RED_MIDDLE;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetSquarePress()){
                GameField.startPosition = GameField.START_POSITION.BLUE_MIDDLE;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }

            telemetry.update();
        }

        autoOption = AUTO_OPTION.PRELOAD_AND_PARK;

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addLine("Input After Drop Purple Pixel Wait");
            telemetry.addLine("(Use dpad_up to increase, and dpad_down to decrease)");
            telemetry.addLine("(Press B / O  to finalize");
            telemetry.addData("    After Drop Purple Pixel Wait ", afterPurplePixelWait);
            if (gamepadController.gp1GetDpad_upPress()) {
                afterPurplePixelWait++;
            }
            if (gamepadController.gp1GetDpad_downPress()) {
                if (afterPurplePixelWait != 0) afterPurplePixelWait--;
            }
            if (gamepadController.gp1GetCirclePress()) {
                break;
            }
            telemetry.update();
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addData("Selected After Drop Purple Pixel Wait", afterPurplePixelWait);
            telemetry.addLine("Input After Drop Yellow Pixel Wait");
            telemetry.addLine("(Use dpad_up to increase, and dpad_down to decrease)");
            telemetry.addLine("(Press B / O  to finalize");
            telemetry.addData("    After Drop Yellow Pixel Wait ", afterYellowPixelWait);
            if (gamepadController.gp1GetDpad_upPress()) {
                afterYellowPixelWait++;
            }
            if (gamepadController.gp1GetDpad_downPress()) {
                if (afterYellowPixelWait != 0) afterYellowPixelWait--;
            }
            if (gamepadController.gp1GetCirclePress()) {
                break;
            }
            telemetry.update();
        }



        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addData("Selected After Drop Purple Pixel Wait", afterPurplePixelWait);
            telemetry.addData("Selected After Drop Yellow Pixel Wait", afterYellowPixelWait);
            telemetry.addLine("Select Parking option");
            telemetry.addData("    Corner Wall Rigging ", "Y / Δ");
            telemetry.addData("    Front of Wall Rigging","(X / ▢)");
            telemetry.addData("    Front of Middle tile   ", "B / O ");
            if (gamepadController.gp1GetTrianglePress()) {
                parkingOption = PARKING_OPTION.CORNER_WALL_RIGGING;
                break;
            }
            if (gamepadController.gp1GetSquarePress()) {
                parkingOption = PARKING_OPTION.FRONT_OF_WALL_RIGGING;
                parkingArm.deploy = true;
                break;
            }
            if (gamepadController.gp1GetCirclePress()) {
                parkingOption = PARKING_OPTION.FRONT_OF_BACKDROP;
                parkingArm.deploy = true;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public void initSubsystems(){

        //telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        intake = new Intake(hardwareMap, telemetry);
        telemetry.addLine("Intake Initialized");
        telemetry.update();

        magazine = new Magazine(hardwareMap, telemetry);
        telemetry.addLine("Magazine Initialized");
        telemetry.update();

        outtakeArm = new OuttakeArm(hardwareMap, telemetry);
        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap, telemetry);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        climber = new Climber(hardwareMap, telemetry);
        telemetry.addLine("Climber Initialized");
        telemetry.update();

        launcher = new Launcher(hardwareMap, telemetry);
        telemetry.addLine("Launcher Initialized");
        telemetry.update();

        parkingArm = new ParkingArm(hardwareMap, telemetry);
        telemetry.addLine("ParkingArm Initialized");
        telemetry.update();

        /* Create VisionSensor */
        visionSensor = new VisionSensor(hardwareMap, telemetry);
        telemetry.addLine("Vision Sensor Initialized");
        telemetry.update();

        /* Create Vision */
        //visionTfodFront = new VisionTfod(hardwareMap, telemetry, "Webcam 1");
        //telemetry.addLine("VisionTfod Initialized");
        visionOpenCV = new VisionOpenCV(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("VisionOpenCV Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);
        telemetry.update();

        outtakeController = new OuttakeController(this.outtakeSlides, this.outtakeArm, this);
        telemetry.addLine("Outtake Controller Initialized");
        telemetry.update();

        intakeController = new IntakeController(this.intake, this.magazine,this);
        telemetry.addLine("Intake Controller Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, intake, magazine,
                outtakeSlides, outtakeArm, climber, launcher, visionSensor, lights, telemetry, this);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            driveTrain.pose = GameField.currentPose;
        } else {
            driveTrain.pose = startPose;
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

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
        //telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat Autonomous Mode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            telemetry.addData("PoseEstimateString :", toStringPose2d(drive.pose));

            driveTrain.printDebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }
}   // end class
