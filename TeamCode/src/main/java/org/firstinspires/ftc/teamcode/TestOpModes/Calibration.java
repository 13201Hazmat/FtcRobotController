package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "Calibration", group = " 01-Testing")
public class Calibration extends LinearOpMode {

    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            gamepadController.runByGamepadControl();

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            if (gamepadController.gp1GetDpad_rightPress()) {
                intakeArm.moveGripForward();
            }
            if (gamepadController.gp1GetDpad_leftPress()) {
                intakeArm.moveGripBackward();
            }

            intakeArm.SWIVEL_DELTA = 0.01;
            if (gamepadController.gp1GetRightTriggerPress()) {
                intakeArm.moveSwivelForward();
            }
            if (gamepadController.gp1GetLeftTriggerPress()) {
                intakeArm.moveSwivelBackward();
            }

            if (gamepadController.gp1GetDpad_upPress()) {
                intakeArm.moveWristForward();
            }
            if (gamepadController.gp1GetDpad_downPress()) {
                intakeArm.moveWristBackward();
            }

            if (gamepadController.gp1GetRightBumperPress()) {
                intakeArm.moveArmForward();
            }
            if (gamepadController.gp1GetLeftBumperPress()) {
                intakeArm.moveArmBackward();
            }

            intakeSlides.INTAKE_SLIDE_DELTA = 0.005;
            if (!gamepadController.gp1GetStart() && gamepadController.gp1GetCirclePress()) {
                intakeSlides.moveIntakeSlideLeftForward();
            }
            if (gamepadController.gp1GetSquarePress()) {
                intakeSlides.moveIntakeSlideLeftBackward();
            }

            if (gamepadController.gp1GetTrianglePress()) {
                intakeSlides.moveIntakeSlideRightForward();
            }
            if (!gamepadController.gp1GetStart() && gamepadController.gp1GetCrossPress()) {
                intakeSlides.moveIntakeSlideRightBackward();
            }

            if (!gamepadController.gp2GetStart() && gamepadController.gp2GetCirclePress()) {
                intakeSlides.moveIntakeSlideLeftForward();
                intakeSlides.moveIntakeSlideRightForward();
            }
            if (gamepadController.gp2GetSquarePress()) {
                intakeSlides.moveIntakeSlideLeftBackward();
                intakeSlides.moveIntakeSlideRightBackward();
            }

            if (gamepadController.gp2GetLeftTriggerPress()){
                outtake.leftPTOServo.setPosition(outtake.leftPTOServo.getPosition() - 0.01);
                outtake.rightPTOServo.setPosition(outtake.rightPTOServo.getPosition() + 0.01);
            }

            if (gamepadController.gp2GetRightTriggerPress()){
                outtake.leftPTOServo.setPosition(outtake.leftPTOServo.getPosition() + 0.01);
                outtake.rightPTOServo.setPosition(outtake.rightPTOServo.getPosition() - 0.01);
            }

            if (gamepadController.gp2GetDpad_rightPress()) {
                outtake.moveGripForward();
            }
            if (gamepadController.gp2GetDpad_leftPress()) {
                outtake.moveGripBackward();
            }


            if (gamepadController.gp2GetDpad_upPress()) {
                outtake.moveWristForward();
            }
            if (gamepadController.gp2GetDpad_downPress()) {
                outtake.moveWristBackward();
            }


            if (gamepadController.gp2GetRightBumperPress()) {
                outtake.moveArmForward();
            }
            if (gamepadController.gp2GetLeftBumperPress()) {
                outtake.moveArmBackward();
            }

            if (gamepadController.gp2GetRightStickButtonPress()) {
                outtake.openGrip();
                outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
                outtake.moveWrist(Outtake.ARM_STATE.TRANSFER);
                intakeArm.loosenGrip();
                intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
                intakeArm.moveArm(IntakeArm.ARM_STATE.TRANSFER);
                intakeArm.moveWristAndSwivel(IntakeArm.ARM_STATE.TRANSFER);
            }

            if (gamepadController.gp2GetLeftStickButtonPress()) {
                outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_CHAMBER);
                outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
                outtake.moveWrist(Outtake.ARM_STATE.HIGH_CHAMBER);
            }

            if (gamepadController.gp2GetTrianglePersistent() ||
                    gamepadController.gp2GetCrossPersistent()) {
                while (gamepadController.gp2GetTrianglePersistent() && !isStopRequested()) {
                    outtake.outtakeSlideLeft.setTargetPosition(outtake.outtakeSlideLeft.getCurrentPosition() + 100);
                    outtake.outtakeSlideRight.setTargetPosition(outtake.outtakeSlideRight.getCurrentPosition() + 100);
                    //outtake.outtakeSlideLeftClimb.setTargetPosition(outtake.outtakeSlideLeftClimb.getCurrentPosition() + 100);
                    //outtake.outtakeSlideRightClimb.setTargetPosition(outtake.outtakeSlideRightClimb.getCurrentPosition() + 100);
                    outtake.outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    outtake.outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    //outtake.outtakeSlideLeftClimb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    //outtake.outtakeSlideRightClimb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    outtake.outtakeSlideLeft.setPower(1.0);
                    outtake.outtakeSlideRight.setPower(1.0);
                    //outtake.outtakeSlideLeftClimb.setPower(1.0);
                    //outtake.outtakeSlideRightClimb.setPower(1.0);
                }
                while (gamepadController.gp2GetCrossPersistent() && !isStopRequested()) {
                    outtake.outtakeSlideLeft.setTargetPosition(outtake.outtakeSlideLeft.getCurrentPosition() - 100);
                    outtake.outtakeSlideRight.setTargetPosition(outtake.outtakeSlideRight.getCurrentPosition() - 100);
                    //outtake.outtakeSlideLeftClimb.setTargetPosition(outtake.outtakeSlideLeftClimb.getCurrentPosition() - 100);
                    //outtake.outtakeSlideRightClimb.setTargetPosition(outtake.outtakeSlideRightClimb.getCurrentPosition() - 100);
                    outtake.outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    outtake.outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    //outtake.outtakeSlideLeftClimb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    //outtake.outtakeSlideRightClimb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    outtake.outtakeSlideLeft.setPower(1.0);
                    outtake.outtakeSlideRight.setPower(1.0);
                    //outtake.outtakeSlideLeftClimb.setPower(1.0);
                    //outtake.outtakeSlideRightClimb.setPower(1.0);
                }
            }

            outtake.outtakeSlideLeft.setPower(0);
            outtake.outtakeSlideRight.setPower(0);

            printDebugMessages();

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

        outtake = new Outtake(hardwareMap, telemetry);
        telemetry.addLine("Outtake Initialized");
        telemetry.update();

        intakeArm = new IntakeArm(hardwareMap, telemetry);
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        intakeSlides = new IntakeSlides(hardwareMap, telemetry);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new TestGamepadController(gamepad1, gamepad2, driveTrain, telemetry);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();
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
            //driveTrain.printDebugMessages();
            //intakeArm.printDebugMessages();
            //intakeSlides.printDebugMessages();
            //outtake.printDebugMessages();
            telemetry.addLine("Intake Grip: GP1 + Dpad_right, - Dpad_left");
            telemetry.addLine("    Tight Close with Sample is 0.26, Fully open is 0.72");
            telemetry.addData("    Grip Position", intakeArm.intakeGripServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Intake Swivel: GP1 + Right Triggger, - Left Trigger");
            telemetry.addLine("    Centered is 0.495");
            telemetry.addData("     Swivel Position", intakeArm.intakeSwivelServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Intake Wrist: GP1 + Dpad_up, - Dpad_down");
            telemetry.addLine("     Zero is when Arm fully pushed in when Arm held horizontally");
            telemetry.addData("     Wrist Position", intakeArm.intakeWristServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Intake Arm: GP1 + Right Bumper, - Left Bumper");
            telemetry.addLine("     Zero is Arm perpendicularly down from slides");
            telemetry.addData("     Arm Position", intakeArm.intakeArmServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Intake Slides Left: GP1 + Square, - Circle");
            telemetry.addLine("     Zero is Slides fully in");
            telemetry.addData("     Left Position", intakeSlides.intakeSlideServoLeft.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Intake Slides Right : GP1 + Triangle, - Cross");
            telemetry.addLine("     Zero is Slides fully in");
            telemetry.addData("     Right Position", intakeSlides.intakeSlideServoRight.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Intake Slides Left + Right : GP2 + Square, - Circle");
            telemetry.addLine("     Zero is Slides fully in");
            telemetry.addData("     Left Position", intakeSlides.intakeSlideServoLeft.getPosition());
            telemetry.addData("     Right Position", intakeSlides.intakeSlideServoRight.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Outtake Grip: GP2+ Dpad_right, - Dpad_left");
            telemetry.addLine("    Grip closed tight with Sample is 0.22");
            telemetry.addData("     Grip Position", outtake.outtakeGripServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Outtake Wrist: GP2 + Dpad_up, - Dpad_down");
            telemetry.addLine("    Zero when outtake arm is fully in mechanical limit and wrist parallel to arm");
            telemetry.addData("     Wrist Position", outtake.outtakeWristServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Outtake Arm: GP2 + Right Bumper, - Left Bumper");
            telemetry.addLine("    Zero when outtake arm is fully in mechanical limit");
            telemetry.addData("      Arm Position", outtake.outtakeArmServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Outtake PTO Left + Right : GP2 + Right Trigger, - LeftTrigger");
            telemetry.addData("     Left PTO Position", outtake.leftPTOServo.getPosition());
            telemetry.addData("     Right PTO Position", outtake.rightPTOServo.getPosition());
            telemetry.addLine("----------");

            telemetry.addLine("Outtake Slides: GP2 + Triangle, - Cross");
            telemetry.addData("Outtake Slides Left Position", outtake.outtakeSlideLeft.getCurrentPosition());
            telemetry.addData("Outtake Slides Right Position", outtake.outtakeSlideRight.getCurrentPosition());
            //telemetry.addData("Outtake Slides Left Climb Position", outtake.outtakeSlideLeftClimb.getCurrentPosition());
            //telemetry.addData("Outtake Slides Right Climb Position", outtake.outtakeSlideRightClimb.getCurrentPosition());
            telemetry.addData("Outtake Slides Left Current", outtake.outtakeSlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Outtake Slides Right Current", outtake.outtakeSlideRight.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("Outtake Slides Left Climb Current", outtake.outtakeSlideLeftClimb.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("Outtake Slides Right Climb Current", outtake.outtakeSlideRightClimb.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("----------");
        }
        telemetry.update();
    }

}