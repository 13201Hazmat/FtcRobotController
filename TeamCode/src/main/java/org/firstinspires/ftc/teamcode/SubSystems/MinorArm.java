package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
public class MinorArm {

    public Servo minorClawServo;

    public enum MINOR_CLAW_STATE {
        OPEN,
        CLOSED,
    }

    public Servo minorArmServo;

    public enum MINOR_SERVO_STATE {
        PICKUP,
        LEVEL_1,
        PARKED,
    }

    public MinorArm(HardwareMap hardwareMap) {
        minorArmServo = hardwareMap.servo.get("minor_arm_servo");
        minorClawServo = hardwareMap.servo.get("minor_claw_servo");
    }

    public static int baselineEncoderCount = 0;
    public static final double CLAW_OPEN = 0.0;
    public static final double CLAW_CLOSED = 1.0;
    public MINOR_CLAW_STATE minorClawState = MINOR_CLAW_STATE.OPEN;
    public static double PICKUP_POSITION_COUNT = 0.0;
    public static double LEVEL1_POSITION_COUNT = 0.33;
    public static double PARKED_POSITION_COUNT = 1.0;
    //public static double MINOR_ARM_DELTA_COUNT = 0.05;
    //public double currentMinorArmPositionCount = PARKED_POSITION_COUNT;
    public MINOR_SERVO_STATE currentMinorServoState = MINOR_SERVO_STATE.PARKED;
    public MINOR_SERVO_STATE previousMinorServoState = MINOR_SERVO_STATE.PARKED;

    //public static double ARM_SERVO_POWER = 0.8;

    public void initMinorArm(){
        //turnArmBrakeModeOff();
        minorClawServo.setPosition(CLAW_CLOSED);
        minorClawState = MINOR_CLAW_STATE.OPEN;
        minorArmServo.setPosition(PARKED_POSITION_COUNT);
        minorArmServo.setDirection(Servo.Direction.FORWARD);
        currentMinorServoState = MINOR_SERVO_STATE.PARKED;
        previousMinorServoState = MINOR_SERVO_STATE.PARKED;
    }

    public MINOR_CLAW_STATE getMinorClawState() {
        return minorClawState;
    }

    public MINOR_SERVO_STATE getMinorServoPosition() {
        return currentMinorServoState;
    }

    /*public void resetMinorArm(){
        DcMotor.RunMode runMode = minorArmServo.getMode();
        minorAr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        majorArmMotor.setMode(runMode);
    }

    public void turnArmBrakeModeOn(){
        mArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turnArmBrakeModeOff(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    } */

    public void changeMinorClawState() {
        if ((minorClawState == MINOR_CLAW_STATE.OPEN)) {
            minorClawServo.setPosition(CLAW_CLOSED);
            minorClawState = MINOR_CLAW_STATE.CLOSED;
        } else if ((minorClawState == MINOR_CLAW_STATE.CLOSED)) {
            minorClawServo.setPosition(CLAW_OPEN);
            minorClawState = MINOR_CLAW_STATE.OPEN;
        }
    }

    //change the level of the Arm to Pickup
    public void moveMinorArmPickupPosition() {
        //turnArmBrakeModeOn();
        minorArmServo.setPosition(PICKUP_POSITION_COUNT + baselineEncoderCount);
        //runArmToLevelState = true;
        currentMinorServoState = MINOR_SERVO_STATE.PICKUP;
    }

    //change the level of the arm to Level One
    public void moveMinorArmLevel1Position() {
        //turnArmBrakeModeOn();
        minorArmServo.setPosition(LEVEL1_POSITION_COUNT + baselineEncoderCount);
        //runArmToLevelState = true;
        currentMinorServoState = MINOR_SERVO_STATE.PICKUP;
    }

    //change the level of the arm to the parking
    public void moveMinorArmParkingPosition() {
        //turnArmBrakeModeOn();
        minorArmServo.setPosition(PARKED_POSITION_COUNT + baselineEncoderCount);
        //runArmToLevelState = true;
        currentMinorServoState = MINOR_SERVO_STATE.PICKUP;
    }
    /**
     * Move Minor Arm Slightly Down
     */
    /*public void moveMajorArmSlightlyDown(){
        if ((currentArmPositionCount >= PARKED_POSITION_COUNT) &&
                currentArmPositionCount <= PICKUP_POSITION_COUNT + MAJORARM_DELTA_COUNT){
            turnArmBrakeModeOn();
            currentArmPositionCount = currentArmPositionCount - MAJORARM_DELTA_COUNT;
            majorArmMotor.setTargetPosition(currentArmPositionCount);
            runArmToLevelState = true;
        }
    }

    /**
     * Move Minor Arm Slightly Up
     */
    /*public void moveMajorArmSlightlyUp(){
        if ((currentArmPositionCount < PICKUP_POSITION_COUNT) &&
                currentArmPositionCount >= PARKED_POSITION_COUNT - MAJORARM_DELTA_COUNT){
            turnArmBrakeModeOn();
            currentArmPositionCount = currentArmPositionCount + MAJORARM_DELTA_COUNT;
            majorArmMotor.setTargetPosition(currentArmPositionCount);
            runArmToLevelState = true;
        }
    }
    */

    //change the level of the Arm to Level 1
    public void moveMinorArmUpOne() {
        if ((currentMinorServoState == MINOR_SERVO_STATE.PICKUP)) {
            previousMinorServoState = currentMinorServoState;
            moveMinorArmLevel1Position();
            return;
        }
        if ((currentMinorServoState == MINOR_SERVO_STATE.LEVEL_1)) {
            previousMinorServoState = currentMinorServoState;
            moveMinorArmParkingPosition();
            return;
        }
    }

    //change the level of the Arm by one Down
    public void moveMinorArmDownOne() {
        if ((currentMinorServoState == MINOR_SERVO_STATE.PARKED)) {
            previousMinorServoState = currentMinorServoState;
            moveMinorArmLevel1Position();
            return;
        }

        if ((currentMinorServoState == MINOR_SERVO_STATE.PARKED)) {
            previousMinorServoState = currentMinorServoState;
            moveMinorArmPickupPosition();
            return;
        }
    }

    //TODO: Add code MajorArm Slight drop (reduce by 50 counts, dont change state when left trigger is pressed.

}