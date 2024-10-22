package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "TestIntakeArm", group = "Testing")
public class TestIntakeArm extends LinearOpMode {

    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public Lights lights;

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

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                //gamepadController.runByGamepadControl();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
                }

                /*
                Triangle (move arm-wrist  and slide to pickup)
                Cross (stop intake roller inward, move arm, wrist and slides to init, or transfer - if outtake bucket is in place, else on second click),
                Circle (move arm and wrist to pickup & start intake roller inward, If started - stop intake roller inward)
                Square (move arm and wrist to eject & start intake roller outward, if started - stop outtake roller outward)
                **dpad_left (rotate left)
                **dpad_right (rotate right)
                Start + Dpad_up (rotate arm forward)
                Start + Dpad_down (rotate arm backward)
                Start + Dpad_right (rotate wrist forward)
                Start + Dpad_left (rotate wrist backward)
                */

                //Triangle (move arm-wrist  and slide to pickup)
                if(gamepadController.gp1GetTrianglePress()){
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP);
                    //Move slide to pickup
                }

                //Cross (stop intake roller inward, move arm, wrist and slides to init, or transfer - if outtake bucket is in place, else on second click)
                if(gamepadController.gp1GetCrossPress()){
                    if (intakeArm.intakeArmState == IntakeArm.INTAKE_ARM_STATE.PICKUP) {
                        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
                    } else {
                        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);
                    }
                }

                //Circle (move arm and wrist to pickup & start intake roller inward, If started - stop intake roller inward)
                if(gamepadController.gp1GetCirclePress()){
                    if (intakeArm.intakeRollerState != IntakeArm.INTAKE_ROLLER_STATE.STOPPED) {
                        intakeArm.runRollerToIntake();
                    } else {
                        intakeArm.stopRoller();
                    }
                }

                //Square (move arm and wrist to eject & start intake roller outward, if started - stop outtake roller outward)
                if(gamepadController.gp1GetSquarePress()){
                    if (intakeArm.intakeRollerState != IntakeArm.INTAKE_ROLLER_STATE.STOPPED) {
                        intakeArm.runRollerToEject();
                    } else {
                        intakeArm.stopRoller();
                    }
                }

                //Start + Dpad_up (rotate arm forward)
                if (gamepadController.gp1GetStart() && gamepadController.gp1GetDpad_upPress()) {
                    intakeArm.moveArmForward();
                }

                //Start + Dpad_down (rotate arm backward)
                if (gamepadController.gp1GetStart() && gamepadController.gp1GetDpad_downPress()) {
                    intakeArm.moveArmBackward();
                }

                //Start + Dpad_right (rotate wrist forward)
                if (gamepadController.gp1GetStart() && gamepadController.gp1GetDpad_rightPress()) {
                    intakeArm.moveWristForward();
                }

                //Start + Dpad_left (rotate wrist backward)
                if (gamepadController.gp1GetStart() && gamepadController.gp1GetDpad_leftPress()) {
                    intakeArm.moveWristBackward();
                }

            }
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
            driveTrain.printDebugMessages();
            intakeArm.printDebugMessages();
        }
        telemetry.update();
    }

}