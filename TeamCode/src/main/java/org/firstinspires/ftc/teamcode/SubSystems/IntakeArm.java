package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public CRServo intakeRollerServo;

    public boolean intakeActivated = false;
    public boolean reverseIntakeActivated = false;

    public ElapsedTime stackIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime reverseStackIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public enum INTAKE_ROLLER_STATE {
        RUNNING,
        REVERSE,
        STOPPED;

        INTAKE_ROLLER_STATE() {}
    }
    public INTAKE_ROLLER_STATE intakeRollerState = INTAKE_ROLLER_STATE.STOPPED;

    public enum INTAKE_ARM_STATE{
        INIT(0),
        PICKUP(0.2),
        TRANSFER(0.5),
        DROP(0.7);

        private double armPos;
        INTAKE_ARM_STATE(double armPos){
            this.armPos = armPos;
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

    public Telemetry telemetry;
    public IntakeArm(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
        intakeRollerServo = hardwareMap.get(CRServo.class, "intake_roller_servo");
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
            case DROP:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.DROP.wristPosition);
                break;
            case PICKUP:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.PICKUP.wristPosition);
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

    public void runRollerForward(){
        stackIntakeTimer.reset();
        intakeRollerServo.setPower(1);
        intakeActivated = true;
        intakeRollerState = INTAKE_ROLLER_STATE.RUNNING;
    }

    public void runRollerReverse() {
        stackIntakeTimer.reset();
        intakeRollerServo.setPower(-0.8);
        reverseIntakeActivated = true;
        intakeRollerState = INTAKE_ROLLER_STATE.REVERSE;
    }

    public void stopRoller(){
        intakeRollerServo.setPower(0);
        intakeActivated = false;
        intakeRollerState = INTAKE_ROLLER_STATE.STOPPED;
    }

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Intake Arm");
        telemetry.addData("   State", intakeArmState);
        telemetry.addData("   Servo position", intakeArmServo.getPosition());
        telemetry.addLine("Intake Wrist");
        telemetry.addData("   State", intakeWristState);
        telemetry.addData("   Wrist Servo position", intakeWristServo.getPosition());
        telemetry.addLine("Intake Roller");
        telemetry.addData("   State", intakeRollerState);
        telemetry.addLine("=============");
    }
}
