package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class OuttakeArm {
    //Initialization of <outtake arm servo's>
    public Servo outtakeWristServo;
    public Servo outtakeGripServo;
    public Servo outtakeArmLeft;
    public Servo outtakeArmRight;

    public enum OUTTAKE_ARM_STATE{
        TRANSFER(0,1), //TODO : Update
        DROP(0,1),

        READY_FOR_PICKUP(0,0);

        private double leftArmPosition;
        private double rightArmPosition;

        OUTTAKE_ARM_STATE(double leftArmPosition, double rightArmPosition){
            this.leftArmPosition = leftArmPosition;
            this.rightArmPosition = rightArmPosition;
        }
        public double getLeftArmPosition(){
            return leftArmPosition;
        }
        public double getRightArmPosition(){
            return rightArmPosition;
        }
    }

    public OUTTAKE_ARM_STATE outtakeArmState = OUTTAKE_ARM_STATE.TRANSFER;


    //Hand - wrist, grip state declaration
    public enum OUTTAKE_WRIST_STATE {
        WRIST_TRANSFER(0), //UPDATE FOR THIS YEAR!!
        WRIST_DROP(0),
        WRIST_MIN(0),
        WRIST_MAX(0);

        private double wristPosition;

        OUTTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }
    public OUTTAKE_WRIST_STATE outtakeWristState = OUTTAKE_WRIST_STATE.WRIST_TRANSFER;
    public double OUTTAKE_WRIST_DELTA = 0.02; //UP

    //Initialization of GRIP_STATE
    public enum OUTTAKE_GRIP_STATE { //state of the Hand Grip
        OPEN(0.47),
        CLOSED(0.8);

        private double gripPosition;

        OUTTAKE_GRIP_STATE(double gripPosition){
            this.gripPosition = gripPosition;
        }
        public double getGripPosition(){
            return gripPosition;
        }
    }
    public OUTTAKE_GRIP_STATE outtakeGripState = OUTTAKE_GRIP_STATE.CLOSED;

    public Telemetry telemetry;
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist_servo");
        outtakeGripServo = hardwareMap.get(Servo.class, "outtake_grip_servo");

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtake_arm_left");
        outtakeArmRight = hardwareMap.get(Servo.class, "outtake_arm_right");

        initOuttakeArm();
    }

    //initialize outtakeArm
    public void initOuttakeArm() {//UPDATE FOR THIS YEAR!!
        moveArm(OUTTAKE_ARM_STATE.TRANSFER);
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            moveArm(OUTTAKE_ARM_STATE.TRANSFER);
            closeGrip();
        } else {
            moveArm(OUTTAKE_ARM_STATE.READY_FOR_PICKUP);
            openGrip();
        }
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip() {
        outtakeGripServo.setPosition(OUTTAKE_GRIP_STATE.OPEN.gripPosition);
        outtakeGripState = OUTTAKE_GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){ //UPDATE FOR THIS YEAR!!
        moveArm(OUTTAKE_ARM_STATE.TRANSFER);
        moveWrist(OUTTAKE_WRIST_STATE.WRIST_TRANSFER); //0.36
        outtakeGripServo.setPosition(OUTTAKE_GRIP_STATE.CLOSED.gripPosition);
        outtakeGripState = OUTTAKE_GRIP_STATE.CLOSED;
    }

    public void moveWrist(OUTTAKE_WRIST_STATE toWristState){
        outtakeWristServo.setPosition(toWristState.wristPosition);
        outtakeWristState = toWristState;
    }

    public void rotateWrist(double direction) {
        double newWristPosition = outtakeWristServo.getPosition() + direction*0.01;
        if (newWristPosition >=0.0 && newWristPosition <=1.0) {
            outtakeWristServo.setPosition(newWristPosition);
        }
    }

    public void moveArm(OUTTAKE_ARM_STATE toArmState) { //UPDATE FOR THIS YEAR!!
        outtakeArmLeft.setPosition(toArmState.leftArmPosition);
        outtakeArmRight.setPosition(toArmState.rightArmPosition);
        outtakeArmState = toArmState;
        /*if(outtakeArmState == OUTTAKE_ARM_STATE.TRANSFER){
            moveWrist(OUTTAKE_WRIST_STATE.WRIST_TRANSFER);
            openGrip();
        }*/

        switch (outtakeArmState) { //UPDATE FOR THIS YEAR!!
            case TRANSFER:
                moveWrist(OUTTAKE_WRIST_STATE.WRIST_TRANSFER);
                openGrip();
                break;
            case DROP:
                moveWrist(OUTTAKE_WRIST_STATE.WRIST_DROP);
                break;
        }
    }

    public void rotateArm(double direction) {
        double newPositionLeft = outtakeArmLeft.getPosition() + direction *0.01;
        double newPositionRight = outtakeArmRight.getPosition() - direction *0.01;
        if (newPositionLeft >= 0.0 && newPositionLeft <= 1.0 &&
                newPositionRight >= 0.0 && newPositionRight <= 1.0) {
            outtakeArmLeft.setPosition(newPositionLeft);
            outtakeArmRight.setPosition(newPositionRight);
        }
    }


    public void printDebugMessages() {
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }
}