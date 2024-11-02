package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public CRServo intakeRollerServo;
    public NormalizedColorSensor intakeSensor;

    public boolean intakeActivated = false;
    public boolean reverseIntakeActivated = false;

    public ElapsedTime intakeRollerTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime reverseIntakeRollerTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public enum INTAKE_ROLLER_STATE {
        RUNNING,
        REVERSE,
        STOPPED;
    }
    public INTAKE_ROLLER_STATE intakeRollerState = INTAKE_ROLLER_STATE.STOPPED;

    public enum INTAKE_ARM_STATE{
        //Zero position - Intake arm vertically downward

        LOWEST(0.0), // Perpendiculr to the ground downnwards
        PICKUP(0.32),
        EJECT(0.45),
        INIT(0.68), //vertically up
        TRANSFER(0.77);

        private double armPos;
        INTAKE_ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public INTAKE_ARM_STATE intakeArmState = INTAKE_ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum INTAKE_WRIST_STATE{
        //Zero position - Horizontallu Facing inward, with Intake Arm in Vertically upward position
        PICKUP(0.092),
        EJECT(0.67),
        TRANSFER(0.16),
        INIT(0.0);

        private final double wristPosition;
        INTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
    }
    public INTAKE_WRIST_STATE intakeWristState = INTAKE_WRIST_STATE.INIT;
    public double WRIST_UP_DELTA = 0.01;

    public Telemetry telemetry;
    public IntakeArm(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
        intakeRollerServo = hardwareMap.get(CRServo.class, "intake_roller_servo");
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intake_sensor");

        initIntakeArm();
    }

    public void initIntakeArm(){
        moveArm(INTAKE_ARM_STATE.INIT);
    }

    public void moveArm(INTAKE_ARM_STATE intakeArmState){
        intakeArmServo.setPosition(intakeArmState.armPos);
        moveWrist(intakeArmState);
        this.intakeArmState = intakeArmState;
    }


    public void moveWrist(INTAKE_ARM_STATE intakeArmState){
        switch (intakeArmState){
            case INIT:
            case LOWEST:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.INIT.wristPosition);
                break;
            case EJECT:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.EJECT.wristPosition);
                break;
            case PICKUP:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.PICKUP.wristPosition);
                break;
            case TRANSFER:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.TRANSFER.wristPosition);
                break;
        }
    }

    public void moveArmForward(){
            intakeArmServo.setPosition(intakeArmServo.getPosition() + WRIST_UP_DELTA);
    }

    public void moveArmBackward(){
        intakeArmServo.setPosition(intakeArmServo.getPosition() - WRIST_UP_DELTA);
    }

    public void moveArmOffVision(){

    }

    public void moveWristForward(){
            intakeWristServo.setPosition(intakeWristServo.getPosition() + WRIST_UP_DELTA);
    }

    public void moveWristBackward(){
            intakeWristServo.setPosition(intakeWristServo.getPosition() - WRIST_UP_DELTA);
    }

    public void runRollerToIntake(){
        intakeRollerTimer.reset();
        intakeRollerServo.setPower(1);
        intakeActivated = true;
        intakeRollerState = INTAKE_ROLLER_STATE.RUNNING;
    }

    public void runRollerToEject() {
        intakeRollerTimer.reset();
        intakeRollerServo.setPower(-1);
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
