package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Intake", group = "Test")
public class TestIntake extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public Intake intake;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //TODO: Declare subsystem to be tested
        intake = new Intake(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        intake.initIntake();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadTestController.runByGamepadControl();

                //TODO: Add Test Code here
                if (gamepadTestController.getDpad_downPress()) {
                    //TODO: Check this logic.. the expectation is : when button is pressed, and if motor is not stopped, stop the motor; and if motor is stopped, start the motor
                    intake.startForwardIntakeMotor();
                    if(intake.getIntakeMotorState() == Intake.INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING) {
                    } else if(intake.getIntakeMotorState() == Intake.INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED) {
                        intake.stopIntakeMotor();
                    }
                }

                //Reverse Intake motors and run - in case of stuck state)
                //TODO: The persisitant button was not liked by our drivers.. change this to same logic as dpad_downPress.. except now it is on dpad_upPress and direction is REVERSING
                if (gamepadTestController.getDpad_upPersistent()) {
                    intake.startReverseIntakeMotor();
                } else if (intake.getIntakeMotorState() == Intake.INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING){
                    intake.stopIntakeMotor();
                }

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        GameField.poseSetInAutonomous = false;
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power : ", driveTrain.getBatteryVoltage(hardwareMap));

        telemetry.addData("Intake State : ", intake.getIntakeMotorState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
