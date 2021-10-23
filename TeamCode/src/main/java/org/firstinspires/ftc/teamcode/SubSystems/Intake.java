package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runIntakeMotor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class Intake {

    public DcMotor intakeMotor = null;


    public enum INTAKE_MOTOR_STATE {
        INTAKE_MOTOR_RUNNING,
        INTAKE_MOTOR_STOPPED,
        INTAKE_MOTOR_REVERSING
    }

    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;

    public double intakeMotorPower1 = 0.95;//0.9;
    public double intakeMotorPower2 = 0.8;

    /*public enum INTAKE_BUTTON_STATE {
        ON,
        OFF
    }*/
    //public INTAKE_BUTTON_STATE intakeButtonState;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        //intakeServo = hardwareMap.servo.get("servotest");
    }

    public void initIntake(){

    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void startForwardIntakeMotor() {
        //TODO: Correct the function name or direction.. Forward = RUNNING, Reverse = REVERSING
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING) {
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower1);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED) {
            runIntakeMotor(DcMotor.Direction.FORWARD, 0.0);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;
       }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void startReverseIntakeMotor() {
        //TODO: Correct the function name or direction.. Forward = RUNNING, Reverse = REVERSING
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING) {
            runIntakeMotor(DcMotor.Direction.FORWARD, intakeMotorPower2);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING;
        }
    }

    private void runIntakeMotor(DcMotor.Direction direction, double power){
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(power);
    }


    /**
     * Returns Intake motor state
     */
    public INTAKE_MOTOR_STATE getIntakeMotorState() {
        return intakeMotorState;
    }
}