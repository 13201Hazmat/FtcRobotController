package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    //public CRServo intakeRollerServo;
    public Servo intakeGripServo;
    public Servo intakeSwivelServo;
    //public NormalizedColorSensor intakeSensor;

    public enum INTAKE_GRIP_STATE {
        OPEN(0.59),
        CLOSED(0.17);

        private final double gripPosition;
        INTAKE_GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public INTAKE_GRIP_STATE intakeGripState = INTAKE_GRIP_STATE.CLOSED;

    public boolean intakeGripAutoClose = true;

    public enum INTAKE_ARM_STATE{
        //Zero position - Intake arm vertically downward

        LOWEST(0.33), // Perpendicular to the ground downwards
        PRE_PICKUP(0.38),
        PICKUP(0.32),//0.34
        EJECT_OR_PRE_TRANSFER(0.38),
        POST_TRANSFER (0.55),
        INIT(0.62), //vertically up
        TRANSFER(0.60),
        SPECIMEN_PICKUP(0.66),
        DYNAMIC(0.68);

        private double armPos;
        INTAKE_ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public INTAKE_ARM_STATE intakeArmState = INTAKE_ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum INTAKE_WRIST_STATE{
        //Zero position - Horizontallu Facing inward, with Intake Arm in Vertically upward position
        PICKUP(0.94),
        EJECT(0.66),
        POST_TRANSFER(0.39),
        PRE_TRANSFER(0.39),
        TRANSFER(0.15),
        INIT(0.22),
        SPECIMEN_PICKUP(0.42),
        DYNAMIC(0.16);

        private final double wristPosition;
        INTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
    }
    public INTAKE_WRIST_STATE intakeWristState = INTAKE_WRIST_STATE.INIT;
    public double WRIST_UP_DELTA = 0.01;

    public enum INTAKE_SWIVEL_STATE{
        //Zero position - Grip Facing center, with specimen held vertical
        LEFT180(0.235),
        CENTERED(0.505),
        RIGHT180(0.785),
        DYNAMIC(0.515);

        private final double swivelPosition;
        INTAKE_SWIVEL_STATE(double swivelPosition){
            this.swivelPosition = swivelPosition;
        }
    }
    public INTAKE_SWIVEL_STATE intakeSwivelState = INTAKE_SWIVEL_STATE.CENTERED;
    public double SWIVEL_DELTA = 0.135;

    public Telemetry telemetry;
    public IntakeArm(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
        //intakeRollerServo = hardwareMap.get(CRServo.class, "intake_roller_servo");
        intakeGripServo = hardwareMap.get(Servo.class, "intake_grip");
        intakeSwivelServo = hardwareMap.get(Servo.class, "intake_swivel");
        //intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intake_sensor");

        initIntakeArm();
    }

    public void initIntakeArm(){
        moveArm(INTAKE_ARM_STATE.TRANSFER);
        intakeArmState = INTAKE_ARM_STATE.TRANSFER;
        openGrip();
    }

    public void moveArm(INTAKE_ARM_STATE toIntakeArmState){
        intakeArmServo.setPosition(toIntakeArmState.armPos);
        moveWristAndSwivel(toIntakeArmState);
        intakeArmState = toIntakeArmState;
    }


    public void moveWristAndSwivel(INTAKE_ARM_STATE intakeArmState){
        switch (intakeArmState){
            case INIT:
            case LOWEST:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.PRE_TRANSFER.wristPosition);
                intakeWristState = INTAKE_WRIST_STATE.PRE_TRANSFER;
                moveSwivelCentered();
                break;
            case EJECT_OR_PRE_TRANSFER:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.EJECT.wristPosition);
                intakeWristState = INTAKE_WRIST_STATE.EJECT;
                break;
            case PRE_PICKUP:
            case PICKUP:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.PICKUP.wristPosition);
                intakeWristState = INTAKE_WRIST_STATE.PICKUP;
                break;
            case TRANSFER:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.TRANSFER.wristPosition);
                intakeWristState = INTAKE_WRIST_STATE.TRANSFER;
                moveSwivelCentered();
                break;
            case POST_TRANSFER:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.POST_TRANSFER.wristPosition);
                intakeWristState = INTAKE_WRIST_STATE.POST_TRANSFER;
                moveSwivelCentered();
                break;
            case SPECIMEN_PICKUP:
                intakeWristServo.setPosition(INTAKE_WRIST_STATE.SPECIMEN_PICKUP.wristPosition);
                intakeWristState = INTAKE_WRIST_STATE.SPECIMEN_PICKUP;
                moveSwivelCentered();
                break;
        }
    }

    public boolean isIntakeArmInSafeStateToMoveOuttake(){
        if (intakeArmState == INTAKE_ARM_STATE.TRANSFER ||
            intakeArmState == INTAKE_ARM_STATE.SPECIMEN_PICKUP ||
            intakeArmState == INTAKE_ARM_STATE.INIT) {
            return false;
        } else {
            return true;
        }
    }

    public void moveArmForward(){
        intakeArmServo.setPosition(intakeArmServo.getPosition() + WRIST_UP_DELTA);
        intakeArmState = INTAKE_ARM_STATE.DYNAMIC;
    }

    public void moveArmBackward(){
        intakeArmServo.setPosition(intakeArmServo.getPosition() - WRIST_UP_DELTA);
        intakeArmState = INTAKE_ARM_STATE.DYNAMIC;
    }

    public void moveArmOffVision(){

    }

    public void moveSwivelCentered(){
        intakeSwivelServo.setPosition(INTAKE_SWIVEL_STATE.CENTERED.swivelPosition);
        intakeSwivelState = INTAKE_SWIVEL_STATE.CENTERED;
    }

    public void moveSwivelPerpendicular(){
        intakeSwivelServo.setPosition(INTAKE_SWIVEL_STATE.LEFT180.swivelPosition);
        intakeSwivelState = INTAKE_SWIVEL_STATE.LEFT180;
    }

    public void toggleSwivel(){
        if (intakeSwivelState == INTAKE_SWIVEL_STATE.CENTERED) {
            moveSwivelPerpendicular();
        } else {
            moveSwivelCentered();
        }
    }


    public void moveSwivelLeft(){
        double intakeSwivelServoPosition = intakeSwivelServo.getPosition();
        if (intakeSwivelServoPosition - SWIVEL_DELTA > INTAKE_SWIVEL_STATE.LEFT180.swivelPosition - 0.02) {
            intakeSwivelServo.setPosition(intakeSwivelServo.getPosition() - SWIVEL_DELTA);
            intakeSwivelState = INTAKE_SWIVEL_STATE.DYNAMIC;
        }
    }

    public void moveSwivelRight(){
        double intakeSwivelServoPosition = intakeSwivelServo.getPosition();
        if (intakeSwivelServoPosition + SWIVEL_DELTA < INTAKE_SWIVEL_STATE.RIGHT180.swivelPosition + 0.02) {
            intakeSwivelServo.setPosition(intakeSwivelServo.getPosition() + SWIVEL_DELTA);
            intakeSwivelState = INTAKE_SWIVEL_STATE.DYNAMIC;
        }
    }

    public void moveWristForward(){
            intakeWristServo.setPosition(intakeWristServo.getPosition() + WRIST_UP_DELTA);
            intakeWristState = INTAKE_WRIST_STATE.DYNAMIC;
    }

    public void moveWristBackward(){
            intakeWristServo.setPosition(intakeWristServo.getPosition() - WRIST_UP_DELTA);
            intakeWristState = INTAKE_WRIST_STATE.DYNAMIC;
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

    public void toggleGrip(){
        if (intakeGripState == INTAKE_GRIP_STATE.CLOSED) {
            openGrip();
        } else {
            closeGrip();
        }
    }


    /*public void runRollerToIntake(){
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

     */

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Intake Arm");
        telemetry.addData("   State", intakeArmState);
        telemetry.addData("   Servo position", intakeArmServo.getPosition());
        telemetry.addLine("Intake Wrist");
        telemetry.addData("   State", intakeWristState);
        telemetry.addData("   Wrist Servo position", intakeWristServo.getPosition());
//        telemetry.addLine("Intake Roller");
//        telemetry.addData("   State", intakeRollerState);
        telemetry.addLine("Intake Swivel");
        telemetry.addData("   State", intakeSwivelState);
        telemetry.addData("   Swivel Servo position", intakeSwivelServo.getPosition());
        telemetry.addLine("Intake Grip");
        telemetry.addData("   State", intakeGripState);
        telemetry.addData("   Grip Servo position", intakeGripServo.getPosition());
        telemetry.addData("   autoClose", intakeGripAutoClose);
        telemetry.addLine("=============");
    }
}
