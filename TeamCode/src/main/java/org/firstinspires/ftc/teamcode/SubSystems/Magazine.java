package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Magazine Class <BR>
 *
 * Example : Magazine consists of system provided magazine controls and adds functionality to the selection made on magazine. <BR>
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
public class Magazine {
    

    //TODO: Update code as needed for Magazine

    public Servo magazineServo = null;

    public static final double MAGAZINE_SERVO_PARKED = 0;
    public static final double MAGAZINE_SERVO_FLIPPED = 0.65;

    public double magazineServoState = MAGAZINE_SERVO_PARKED;

    public enum MAGAZINE_SERVO_STATE {
        PARKED,
        FLIPPED
    }

    public enum MAGAZINE_BUTTON_STATE {
        ON,
        OFF
    }
    public MAGAZINE_BUTTON_STATE magazineButtonState;

    public Magazine(HardwareMap hardwareMap) {
        magazineServo = hardwareMap.servo.get("magazine_servo");
    }

    public void initMagazine(){
        magazineServo.setDirection(Servo.Direction.FORWARD);
        moveMagazineServoToParked();
    }

    /**
     * Sets magazineServo to parked position
     */
    public void moveMagazineServoToParked(){
        magazineServo.setPosition(MAGAZINE_SERVO_PARKED);
        magazineServoState = MAGAZINE_SERVO_PARKED;
    }

    /**
     * set
     */
    public void moveMagazineServoToFlipped(){
        magazineServo.setPosition(MAGAZINE_SERVO_FLIPPED);
        magazineServoState = (int) MAGAZINE_SERVO_FLIPPED;
    }

    /**
     * Returns Magazine servo state
     * @return
     */
    public double getMagazineServoState() {
        return magazineServoState;
    }
}