package org.firstinspires.ftc.teamcode.SubSystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.vision.opencv.ColorRange;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public Servo intakeGripServo;
    public Servo intakeSwivelServo;
    public NormalizedColorSensor intakeSensor;

    public enum GRIP_STATE {
        OPEN_WIDE(0.82), //0.50 max
        OPEN(0.67),//0.36
        LOOSENED(0.45),//0.15
        CLOSED(0.42);//0.11

        private final double gripPosition;
        GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public GRIP_STATE intakeGripState = GRIP_STATE.CLOSED;
    public double GRIP_DELTA = 0.01;

    //public boolean intakeGripAutoClose = true;

    public enum ARM_STATE {
        //Zero position - Intake arm vertically downward
        //Vertical up is 0.66

        LOWEST(0.33), // Perpendicular to the ground downwards
        PRE_PICKUP(0.40), //0.42
        PICKUP(0.27),//0.32
        POST_PICKUP(0.40),//0.34
        INSPECTION(0.35),
        EJECT_OR_PRE_TRANSFER(0.35),//0.38
        POST_TRANSFER (0.52),
        INIT(0.60), //vertically up
        TRANSFER(0.59), //0.66
        LOWER_PRE_PICKUP(0.36), //0.6
        SPECIMEN_PICKUP(0.73),
        DYNAMIC(0.68);

        private double armPos;
        ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public ARM_STATE intakeArmState = ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum WRIST_STATE {
        //Zero position - Horizontallu Facing inward, with Intake Arm in Vertically upward position
        //Go to pick up position and wrist should be vertically down
        PICKUP(0.88),//0.94
        EJECT(0.56),//0.66
        INSPECTION(0.56),
        POST_PICKUP(0.56),//0.66
        POST_TRANSFER(0.38),//0.39
        TRANSFER(0.21), //0.1
        SWEEP(0.61),
        INIT(0.16),//0.22
        SPECIMEN_PICKUP(0.01),//0.88
        DYNAMIC(0.16);

        private final double wristPosition;
        WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
    }
    public WRIST_STATE intakeWristState = WRIST_STATE.INIT;
    public double WRIST_DELTA = 0.01;

    public enum SWIVEL_STATE {
        //Zero position - Grip Facing center, with specimen held vertical
        LEFT180(0.225),
        CENTERED(0.495),
        RIGHT180(0.775),
        DYNAMIC(0.495);

        private final double swivelPosition;
        SWIVEL_STATE(double swivelPosition){
            this.swivelPosition = swivelPosition;
        }
    }
    public SWIVEL_STATE intakeSwivelState = SWIVEL_STATE.CENTERED;
    public double SWIVEL_DELTA = 0.135;

    public Telemetry telemetry;
    public IntakeArm(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
        intakeGripServo = hardwareMap.get(Servo.class, "intake_grip");
        intakeSwivelServo = hardwareMap.get(Servo.class, "intake_swivel");
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intake_sensor");

        initIntakeArm();
    }

    public void initIntakeArm(){
        moveArm(ARM_STATE.INIT);
        moveWristAndSwivel(ARM_STATE.INIT);
        intakeArmState = ARM_STATE.INIT;
        openGrip();
        if (intakeSensingActivated) {
            if (intakeSensor instanceof SwitchableLight) {
                ((SwitchableLight) intakeSensor).enableLight(true);
            }
            intakeSensor.setGain(2);
        } else {
            if (intakeSensor instanceof SwitchableLight) {
                ((SwitchableLight) intakeSensor).enableLight(false);
            }
        }
    }

    public void moveArm(ARM_STATE toIntakeArmState){
        moveWristAndSwivel(toIntakeArmState);
        intakeArmServo.setPosition(toIntakeArmState.armPos);
        intakeArmState = toIntakeArmState;
    }


    public void moveWristAndSwivel(ARM_STATE intakeArmState){
        switch (intakeArmState){
            case INIT:
            case LOWEST:
                intakeWristServo.setPosition(WRIST_STATE.INIT.wristPosition);
                intakeWristState = WRIST_STATE.INIT;
                moveSwivelCentered();
                break;
            case INSPECTION:
                intakeWristServo.setPosition(WRIST_STATE.INSPECTION.wristPosition);
                intakeWristState = WRIST_STATE.INSPECTION;
                moveSwivelCentered();
                break;
            case EJECT_OR_PRE_TRANSFER:
                intakeWristServo.setPosition(WRIST_STATE.EJECT.wristPosition);
                intakeWristState = WRIST_STATE.EJECT;
                break;
            case PRE_PICKUP:
            case PICKUP:
                intakeWristServo.setPosition(WRIST_STATE.PICKUP.wristPosition);
                intakeWristState = WRIST_STATE.PICKUP;
                break;
            case POST_PICKUP:
                intakeWristServo.setPosition(WRIST_STATE.POST_PICKUP.wristPosition);
                moveSwivelPerpendicular();
                intakeWristState = WRIST_STATE.POST_PICKUP;
                break;
            case TRANSFER:
                intakeWristServo.setPosition(WRIST_STATE.TRANSFER.wristPosition);
                intakeWristState = WRIST_STATE.TRANSFER;
                moveSwivelCentered();
                intakeGripServo.setPosition(GRIP_STATE.LOOSENED.gripPosition);
                intakeGripState = GRIP_STATE.LOOSENED;
                break;
            case POST_TRANSFER:
                intakeWristServo.setPosition(WRIST_STATE.POST_TRANSFER.wristPosition);
                intakeWristState = WRIST_STATE.POST_TRANSFER;
                moveSwivelCentered();
                break;
            case LOWER_PRE_PICKUP:
                intakeWristServo.setPosition(WRIST_STATE.SPECIMEN_PICKUP.wristPosition);
                intakeWristState = WRIST_STATE.SPECIMEN_PICKUP;
                //moveSwivelCentered();
                break;
        }
    }

    public boolean isIntakeArmInSafeStateToMoveOuttake(){
        if (intakeArmState == ARM_STATE.TRANSFER ||
            intakeArmState == ARM_STATE.INIT) {
            return false;
        } else {
            return true;
        }
    }

    public void moveArmForward(){
        intakeArmServo.setPosition(intakeArmServo.getPosition() + ARM_DELTA);
        intakeArmState = ARM_STATE.DYNAMIC;
    }

    public void moveArmBackward(){
        intakeArmServo.setPosition(intakeArmServo.getPosition() - ARM_DELTA);
        intakeArmState = ARM_STATE.DYNAMIC;
    }

    public void moveSwivelTo(double servoDegrees){
        intakeSwivelServo.setPosition(SWIVEL_STATE.CENTERED.swivelPosition * (1- servoDegrees/180.0));
        intakeSwivelState = SWIVEL_STATE.DYNAMIC;
    }

    public void moveSwivelCentered(){
        intakeSwivelServo.setPosition(SWIVEL_STATE.CENTERED.swivelPosition);
        intakeSwivelState = SWIVEL_STATE.CENTERED;
    }

    public void moveSwivelPerpendicular(){
        intakeSwivelServo.setPosition(SWIVEL_STATE.LEFT180.swivelPosition);
        intakeSwivelState = SWIVEL_STATE.LEFT180;
    }

    public void toggleSwivel(){
        if (intakeSwivelState == SWIVEL_STATE.CENTERED) {
            moveSwivelPerpendicular();
        } else {
            moveSwivelCentered();
        }
    }

    public void moveSwivelForward(){
            intakeSwivelServo.setPosition(intakeSwivelServo.getPosition() + SWIVEL_DELTA);
    }

    public void moveSwivelBackward(){
        intakeSwivelServo.setPosition(intakeSwivelServo.getPosition() - SWIVEL_DELTA);
    }


    public void moveSwivelLeft(){
        double intakeSwivelServoPosition = intakeSwivelServo.getPosition();
        if (intakeSwivelServoPosition - SWIVEL_DELTA > SWIVEL_STATE.LEFT180.swivelPosition - 0.02) {
            intakeSwivelServo.setPosition(intakeSwivelServo.getPosition() - SWIVEL_DELTA);
            intakeSwivelState = SWIVEL_STATE.DYNAMIC;
        }
    }

    public void moveSwivelRight(){
        double intakeSwivelServoPosition = intakeSwivelServo.getPosition();
        if (intakeSwivelServoPosition + SWIVEL_DELTA < SWIVEL_STATE.RIGHT180.swivelPosition + 0.02) {
            intakeSwivelServo.setPosition(intakeSwivelServo.getPosition() + SWIVEL_DELTA);
            intakeSwivelState = SWIVEL_STATE.DYNAMIC;
        }
    }

    public void moveWristForward(){
            intakeWristServo.setPosition(intakeWristServo.getPosition() + WRIST_DELTA);
            intakeWristState = WRIST_STATE.DYNAMIC;
    }

    public void moveWristBackward(){
            intakeWristServo.setPosition(intakeWristServo.getPosition() - WRIST_DELTA);
            intakeWristState = WRIST_STATE.DYNAMIC;
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS ||
                intakeArmState == ARM_STATE.LOWER_PRE_PICKUP) {
            intakeGripServo.setPosition(GRIP_STATE.OPEN_WIDE.gripPosition);
        } else {
            intakeGripServo.setPosition(GRIP_STATE.OPEN.gripPosition);
        }
        intakeGripState = GRIP_STATE.OPEN;
    }

    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        intakeGripServo.setPosition(GRIP_STATE.CLOSED.gripPosition);
        intakeGripState = GRIP_STATE.CLOSED;
    }

    public void toggleGrip(){
        if (intakeGripState == GRIP_STATE.CLOSED) {
            openGrip();
        } else {
            closeGrip();
        }
    }

    public void loosenGrip(){
        intakeGripServo.setPosition(GRIP_STATE.LOOSENED.gripPosition);
        intakeGripState = GRIP_STATE.LOOSENED;
    }

    public void moveGripForward(){
        intakeGripServo.setPosition(intakeGripServo.getPosition() + GRIP_DELTA);
    }

    public void moveGripBackward(){
        intakeGripServo.setPosition(intakeGripServo.getPosition() - GRIP_DELTA);
    }

    public boolean intakeSensingActivated = true;
    public boolean intakeSampleSensed = false;
    public ColorRange sensedSampleColor = ColorRange.GREEN;
    public double SENSE_DISTANCE = 13;
    public float[] sensedSampleHsvValues = new float[3];
    public NormalizedRGBA sensedColor;
    public double intakeSensingDistance = 500;

    public void senseIntakeSampleColor(){
        if (intakeSensingActivated) {
            if (intakeSensor instanceof DistanceSensor){
                intakeSensingDistance = ((DistanceSensor) intakeSensor).getDistance(DistanceUnit.MM);
            }
            if(intakeSensingDistance < SENSE_DISTANCE){
                intakeSampleSensed = true;
                sensedColor = intakeSensor.getNormalizedColors();
                Color.colorToHSV(sensedColor.toColor(), sensedSampleHsvValues);
            } else {
                intakeSampleSensed = false;
            }

        } else {
            intakeSampleSensed = false;
            sensedSampleColor = ColorRange.GREEN;
        }
    }


    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Intake Arm");
        telemetry.addData("   State", intakeArmState);
        telemetry.addData("   Servo position", intakeArmServo.getPosition());
        telemetry.addLine("Intake Wrist");
        telemetry.addData("   State", intakeWristState);
        telemetry.addData("   Wrist Servo position", intakeWristServo.getPosition());
        telemetry.addLine("Intake Swivel");
        telemetry.addData("   State", intakeSwivelState);
        telemetry.addData("   Swivel Servo position", intakeSwivelServo.getPosition());
        telemetry.addLine("Intake Grip");
        telemetry.addData("   State", intakeGripState);
        telemetry.addData("   Grip Servo position", intakeGripServo.getPosition());
        //telemetry.addData("   autoClose", intakeGripAutoClose);
        telemetry.addLine("Intake Sensor");
        telemetry.addData("   intakeSensingActivated", intakeSensingActivated);
        if (intakeSensingActivated) {
            telemetry.addData("   intakeSensingDistance", intakeSensingDistance);
            telemetry.addData("   intakeSampleSensed", intakeSampleSensed);
            if (intakeSampleSensed) {
                telemetry.addData("    RGB","%.3f, %.3f, %.3f", sensedColor.red, sensedColor.green, sensedColor.blue);
                telemetry.addData("    HSVA","%.3f, %.3f, %.3f, %.3f", sensedSampleHsvValues[0], sensedSampleHsvValues[1], sensedSampleHsvValues[2], sensedColor.alpha);
            }
        }
        telemetry.addLine("=============");
    }
}
