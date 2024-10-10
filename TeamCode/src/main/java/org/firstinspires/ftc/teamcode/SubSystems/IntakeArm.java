package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public Servo intakeGripServo;

    public enum INTAKE_GRIP_STATE { //state of the Hand Grip
        OPEN(0.69),//0.65
        OPEN_AUTO(0.65),
        CLOSED(1.0);

        private final double gripPosition;
        INTAKE_GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public INTAKE_GRIP_STATE intakeGripState = INTAKE_GRIP_STATE.CLOSED;

    public enum INTAKE_ARM_STATE{
        INIT(0),
        TRANSFER(0.5);

        private double armPos;
        //public final int index;
        INTAKE_ARM_STATE(double armPos){
            this.armPos = armPos;
            //this.index = index;
        }
    }
    public INTAKE_ARM_STATE intakeArmState = INTAKE_ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum INTAKE_WRIST_STATE{
        INIT(0),
        PICKUP(0.25),
        DROP(-0.25);

        private final double wristPosition;
        INTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
    }
    public INTAKE_WRIST_STATE intakeWristState = INTAKE_WRIST_STATE.INIT;
    public double wristArmFactor = 1;

    public IntakeArm(HardwareMap hardwareMap) { //map hand servo's to each
        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
        intakeGripServo = hardwareMap.get(Servo.class, "intake_grip_servo");
        initIntakeArm();
    }

    public void initIntakeArm(){
        moveArm(INTAKE_ARM_STATE.INIT);
    }

    public void moveArm(INTAKE_ARM_STATE intakeArmState){
        intakeArmServo.setPosition(intakeArmState.armPos);
        this.intakeArmState = intakeArmState;
    }

    public static final double WRIST_UP_DELTA = 0.2;
    public void moveWrist(INTAKE_ARM_STATE intakeArmState){
        switch (intakeArmState){
            case INIT:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.INIT.wristPosition);
                break;
            case TRANSFER:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.DROP.wristPosition);
                break;
        }
    }

    public void moveWristUp(){
        if (intakeWristState != INTAKE_WRIST_STATE.DROP) {
            intakeWristServo.setPosition(intakeWristServo.getPosition() + WRIST_UP_DELTA);
        }
    }

    public void moveWristDown(){
        if (intakeWristState != INTAKE_WRIST_STATE.DROP) {
            intakeWristServo.setPosition(intakeWristServo.getPosition() - WRIST_UP_DELTA);
        }
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        intakeGripServo.setPosition(INTAKE_GRIP_STATE.OPEN.gripPosition);
        intakeGripState = INTAKE_GRIP_STATE.OPEN;
    }

    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        intakeGripServo.setPosition(INTAKE_GRIP_STATE.CLOSED.gripPosition);
        intakeGripState = INTAKE_GRIP_STATE.CLOSED;
    }

}
