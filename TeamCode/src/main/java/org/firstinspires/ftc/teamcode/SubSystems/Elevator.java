package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runSubsystem1Motor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class Elevator {

    //TODO: Update code as needed for Subsystem1

    public DcMotor elevatorMotor = null;


    public enum ELEVATOR_MOTOR_STATE {
        COLLECT,
        FIRST,
        SECOND,
        THIRD,
        SLIGHTLY_DOWN
    }

    public ELEVATOR_MOTOR_STATE elevatorMotorState = ELEVATOR_MOTOR_STATE.COLLECT;

    public double elevatorMotorPower1 = 0.95;//0.9;
    public static int COLLECT = (int) 0.0;
    public static int FIRST = (int) 0.25;
    public static int SECOND = (int) 0.5;
    public static int THIRD = (int) 0.75;

    public enum ELEVATOR_BUTTON_STATE {
        ON,
        OFF
    }
    public ELEVATOR_BUTTON_STATE elevatorButtonState;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.dcMotor.get("frmotor");
        //subsystem1Servo = hardwareMap.servo.get("servotest");
    }

    public void initElevator(){

    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void startCollectForwardElevatorMotor() {
        if(elevatorMotorState == ELEVATOR_MOTOR_STATE.COLLECT) {
            runElevatorMotor(DcMotor.Direction.REVERSE, elevatorMotor);
            elevatorMotorState = ELEVATOR_MOTOR_STATE.COLLECT;
        }
    }

    private void runElevatorMotor(DcMotorSimple.Direction reverse, DcMotor elevatorMotor) {
    }


    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopFirstElevatorMotor() {
        if(elevatorMotorState != ELEVATOR_MOTOR_STATE.FIRST) {
            runElevatorMotor(DcMotor.Direction.FORWARD, 0.0);
            elevatorMotorState = ELEVATOR_MOTOR_STATE.FIRST;
       }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void startSecondElevatorMotor() {
        if(elevatorMotorState != ELEVATOR_MOTOR_STATE.SECOND) {
            runElevatorMotor(DcMotor.Direction.FORWARD, elevatorMotorPower1);
            elevatorMotorState = ELEVATOR_MOTOR_STATE.SECOND;
        }
    }



    public void startThirdElevatorMotor() {
        if(elevatorMotorState != ELEVATOR_MOTOR_STATE.THIRD) {
            runElevatorMotor(DcMotor.Direction.FORWARD, elevatorMotorPower1);
            elevatorMotorState = ELEVATOR_MOTOR_STATE.THIRD;
        }
    }




    public void startSlightlyDownElevatorMotor() {
        if(elevatorMotorState != ELEVATOR_MOTOR_STATE.SLIGHTLY_DOWN) {
            runElevatorMotor(DcMotor.Direction.FORWARD, elevatorMotorPower1);
            elevatorMotorState = ELEVATOR_MOTOR_STATE.SLIGHTLY_DOWN;
        }
    }

    private void runElevatorMotor(DcMotor.Direction direction, double power){
        elevatorMotor.setDirection(direction);
        elevatorMotor.setPower(power);
    }

    /**
     * set Intake gripper position to hold.. to ensure intake is within robot dimensions at start
     */
    public void setIntakeReleaseHold(){

    }

    /**
     * set Intake gripper position to release
     */
    public void setIntakeReleaseOpen(){

    }

    /**
     * Returns Intake motor state
     */
    public ELEVATOR_MOTOR_STATE getSubsystemMotorState() {
        return elevatorMotorState;
    }
}