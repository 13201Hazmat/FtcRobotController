package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.vision.opencv.ColorRange;

public class Outtake {
    public Servo outtakeArmServo;
    public Servo outtakeWristServo;
    public Servo outtakeGripServo;
    public NormalizedColorSensor outtakeSensor;
    public DcMotorEx outtakeSlideLeft;
    //public DcMotorEx outtakeSlideLeftClimb;
    public DcMotorEx outtakeSlideRight;
    //public DcMotorEx outtakeSlideRightClimb;
    public DigitalChannel outtakeTouch;

    public enum GRIP_STATE {
        OPEN(0.52), //0.59 max
        CLOSED(0.19);

        private final double gripPosition;
        GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public Outtake.GRIP_STATE outtakeGripState = GRIP_STATE.CLOSED;
    public double GRIP_DELTA = 0.01;

    public enum ARM_STATE {
        //Calib Position : Fully in mechanical limit inwards is One
        INIT(0.04),
        PRE_TRANFER(0.06),
        TRANSFER(0.1),
        DROP(0.66),
        HIGH_CHAMBER(0.66),
        MAX(0.66);

        private double armPos;
        ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public ARM_STATE outtakeArmState = ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum WRIST_STATE {
        //Calib Position : Fully in mechanical limit inwards is One
        INIT(0.20),
        PRE_TRANSFER(0.21),
        TRANSFER(0.21),//0.240.55
        //PRE_DROP(0.73),
        HIGH_CHAMBER(0.66),
        DROP(0.68),//0.96
        MAX(0.68);

        private double wristPos;
        WRIST_STATE(double wristPos){
            this.wristPos = wristPos;
        }
    }
    public WRIST_STATE outtakeWristState = WRIST_STATE.TRANSFER;
    public double WRIST_DELTA = 0.01;

    public static double CONVERT_435_TO_1150 = 435.0/1150.0 ;

    //Outtake Motor states
    public enum SLIDE_STATE {
        TRANSFER(0),
        LOW_BUCKET(350), //700 for 223, 508 for 312 (350 for 435rpm motor)
        HIGH_BUCKET(2900), //2400 for 223, 1673 for 312 (2100 for 435rpm motor)
        HIGH_CHAMBER(50), //125 for 223
        CLIMBER2(1800), //1200 for 223, 1000 for 312 ( 1200 for 435rpm motor)
        MAX_EXTENDED(3100); //2800 for 223, 2091 for 312 (2400 for 435rpm motor)

        public final double motorPosition;
        SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;

        }
    }

    public SLIDE_STATE outtakeSlidesState = SLIDE_STATE.TRANSFER;
    public SLIDE_STATE lastOuttakeSlideState = SLIDE_STATE.TRANSFER;

    public boolean climberAscended = false;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 50;//200

    //Different constants of arm speed
    public static final double OUTTAKE_MOTOR_POWER= 1.0;//0.75

    public ElapsedTime outtakeStallTimer = new ElapsedTime(MILLISECONDS);
    public boolean outtakeStallTimingFlag = false;
    public double STALL_TIME = 5000;

    public boolean runOuttakeMotorToLevelState = false;

    Telemetry telemetry;
    public Outtake(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        outtakeArmServo = hardwareMap.get(Servo.class, "outtake_arm");
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist");
        outtakeGripServo = hardwareMap.get(Servo.class, "outtake_grip");
        outtakeSensor = hardwareMap.get(NormalizedColorSensor.class, "outtake_sensor");
        outtakeSlideLeft = hardwareMap.get(DcMotorEx.class, "outtake_slides_left");
        outtakeSlideRight = hardwareMap.get(DcMotorEx.class, "outtake_slides_right");
        //outtakeSlideLeftClimb = hardwareMap.get(DcMotorEx.class, "outtake_climb_left");
        //outtakeSlideRightClimb = hardwareMap.get(DcMotorEx.class, "outtake_climb_right");
        outtakeTouch = hardwareMap.get(DigitalChannel .class, "outtakeTouch");

        initOuttake();
    }

    public void initOuttake(){
        moveArm(ARM_STATE.TRANSFER);
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = SLIDE_STATE.TRANSFER;

        if (outtakeSensingActivated) {
            if (outtakeSensor instanceof SwitchableLight) {
                ((SwitchableLight) outtakeSensor).enableLight(true);
            }
            outtakeSensor.setGain(2);
        } else {
            if (outtakeSensor instanceof SwitchableLight) {
                ((SwitchableLight) outtakeSensor).enableLight(false);
            }
        }

        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            closeGrip();
        }
    }

    public void moveArm(ARM_STATE toOuttakeArmState){
        outtakeArmServo.setPosition(toOuttakeArmState.armPos);
        moveWrist(toOuttakeArmState);
        outtakeArmState = toOuttakeArmState;
    }

    public void moveArmForward(){
        outtakeArmServo.setPosition(outtakeArmServo.getPosition() + ARM_DELTA);
    }

    public void moveArmBackward(){
        outtakeArmServo.setPosition(outtakeArmServo.getPosition() - ARM_DELTA);
    }

    public void moveWrist(ARM_STATE outtakeArmState){
        switch (outtakeArmState){
            case INIT:
                outtakeWristServo.setPosition(WRIST_STATE.INIT.wristPos);
                outtakeWristState = WRIST_STATE.INIT;
                break;
            case PRE_TRANFER:
                outtakeWristServo.setPosition(WRIST_STATE.PRE_TRANSFER.wristPos);
                outtakeWristState = WRIST_STATE.PRE_TRANSFER;
                openGrip();
            case TRANSFER:
                outtakeWristServo.setPosition(WRIST_STATE.TRANSFER.wristPos);
                outtakeWristState = WRIST_STATE.TRANSFER;
                openGrip();
                break;
            case DROP:
                outtakeWristServo.setPosition(WRIST_STATE.DROP.wristPos);
                outtakeWristState = WRIST_STATE.DROP;
                break;
            case HIGH_CHAMBER:
                outtakeWristServo.setPosition(WRIST_STATE.HIGH_CHAMBER.wristPos);
                outtakeWristState = WRIST_STATE.HIGH_CHAMBER;
                break;
            case MAX:
                outtakeWristServo.setPosition(WRIST_STATE.MAX.wristPos);
                outtakeWristState = WRIST_STATE.MAX;
                break;
        }
    }

    public void ascendToClimb(){
        moveOuttakeSlides(SLIDE_STATE.MAX_EXTENDED);
        climberAscended = true;
    }

    public void climb(){
        moveOuttakeSlides(SLIDE_STATE.CLIMBER2);
    }

    public void moveWristForward(){
        outtakeWristServo.setPosition(outtakeWristServo.getPosition() + WRIST_DELTA);
    }

    public void moveWristBackward(){
        outtakeWristServo.setPosition(outtakeWristServo.getPosition() - WRIST_DELTA);
    }

    /*public void moveWristDrop(){
        if (outtakeWristState == WRIST_STATE.PRE_DROP) {
            outtakeWristServo.setPosition(WRIST_STATE.DROP.wristPos);
            outtakeWristState = WRIST_STATE.DROP;
        }
        openGrip();
    }*/

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        outtakeGripServo.setPosition(Outtake.GRIP_STATE.OPEN.gripPosition);
        outtakeGripState = Outtake.GRIP_STATE.OPEN;
    }

    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        outtakeGripServo.setPosition(Outtake.GRIP_STATE.CLOSED.gripPosition);
        outtakeGripState = Outtake.GRIP_STATE.CLOSED;
    }

    public void toggleGrip(){
        if (outtakeGripState == Outtake.GRIP_STATE.CLOSED) {
            openGrip();
        } else {
            closeGrip();
        }
    }

    public void moveGripForward(){
        outtakeGripServo.setPosition(outtakeGripServo.getPosition() + GRIP_DELTA);
    }

    public void moveGripBackward(){
        outtakeGripServo.setPosition(outtakeGripServo.getPosition() - GRIP_DELTA);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        outtakeSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //outtakeSlideLeftClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //outtakeSlideRightClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        outtakeSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //outtakeSlideLeftClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //outtakeSlideRightClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        outtakeSlideLeft.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlideRight.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        //outtakeSlideLeftClimb.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        //outtakeSlideRightClimb.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (outtakeSlidesState == SLIDE_STATE.TRANSFER) {
            turnOuttakeBrakeModeOff();
        } else {
            turnOuttakeBrakeModeOn();
        }
        power = OUTTAKE_MOTOR_POWER;

        outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //outtakeSlideLeftClimb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //outtakeSlideRightClimb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runOuttakeMotorToLevelState == true){
            outtakeSlideLeft.setPower(power);
            outtakeSlideRight.setPower(power);
            //outtakeSlideLeftClimb.setPower(power);
            //outtakeSlideRightClimb.setPower(power);
            runOuttakeMotorToLevelState = false;
        } else{
            outtakeSlideLeft.setPower(0.0);
            outtakeSlideRight.setPower(0.0);
            //outtakeSlideLeftClimb.setPower(0.0);
            //outtakeSlideRightClimb.setPower(0.0);
        }
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runModeOuttakeSlideLeft = outtakeSlideLeft.getMode();
        DcMotorEx.RunMode runModeOuttakeSlideRight = outtakeSlideRight.getMode();
        //DcMotorEx.RunMode runModeOuttakeSlideLeftClimb = outtakeSlideLeftClimb.getMode();
        //DcMotorEx.RunMode runModeOuttakeSlideRightClimb = outtakeSlideRightClimb.getMode();
        outtakeSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //outtakeSlideLeftClimb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //outtakeSlideRightClimb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(runModeOuttakeSlideLeft);
        outtakeSlideRight.setMode(runModeOuttakeSlideRight);
        //outtakeSlideLeftClimb.setMode(runModeOuttakeSlideLeftClimb);
        //outtakeSlideRightClimb.setMode(runModeOuttakeSlideRightClimb);
        outtakeSlideLeft.setPositionPIDFCoefficients(10.0);
        outtakeSlideRight.setPositionPIDFCoefficients(10.0);
        //outtakeSlideLeftClimb.setPositionPIDFCoefficients(10.0);
        //outtakeSlideRightClimb.setPositionPIDFCoefficients(10.0);
        outtakeSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotorEx.Direction.FORWARD);
        //outtakeSlideLeftClimb.setDirection(DcMotorEx.Direction.FORWARD);
        //outtakeSlideRightClimb.setDirection(DcMotorEx.Direction.REVERSE);
    }

    //TODO : Add logic to use Voltage Sensor to measure motor stalling and reset.
    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        outtakeSlideLeft.setTargetPosition((int) (outtakeSlideLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        outtakeSlideRight.setTargetPosition((int) (outtakeSlideRight.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        //outtakeSlideLeftClimb.setTargetPosition((int) (outtakeSlideLeftClimb.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        //outtakeSlideRightClimb.setTargetPosition((int) (outtakeSlideRightClimb.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = SLIDE_STATE.TRANSFER;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(SLIDE_STATE toOuttakeSlideState) {
        //isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesInStateError = Math.abs(outtakeSlideLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 50);
    }

    public double isOuttakeSlidesLeftInStateError = 0;
    public double isOuttakeSlidesRightInStateError = 0;
    public boolean isOuttakeSlidesInTransfer() {
        //isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesLeftInStateError = outtakeSlideLeft.getCurrentPosition();
        isOuttakeSlidesRightInStateError = outtakeSlideRight.getCurrentPosition();
        return (isOuttakeSlidesLeftInStateError < 50 || isOuttakeSlidesRightInStateError < 20);
    }

    public void safetyReset(){
        if (outtakeSlidesState == Outtake.SLIDE_STATE.TRANSFER) {
            if (outtakeTouch.getState() == false) {//PRESSED
                resetOuttakeMotorMode();
                //outtakeStallTimingFlag = false;
            } else {
                if (!outtakeStallTimingFlag) {
                    outtakeStallTimingFlag = true;
                    outtakeStallTimer.reset();
                } else {
                    if (outtakeStallTimer.time() > STALL_TIME) {
                        resetOuttakeMotorMode();
                        outtakeStallTimingFlag = false;
                    }
                }
            }
        } else {
            outtakeStallTimingFlag = false;
        }
    }


    public boolean isOuttakeInTransfer(){
        return (isOuttakeSlidesInState(SLIDE_STATE.TRANSFER) &&
                        outtakeArmState == ARM_STATE.TRANSFER &&
                        outtakeWristState == WRIST_STATE.TRANSFER);
    }

    public boolean isOuttakeReadyToDrop(){
        return ((outtakeArmState == ARM_STATE.DROP || outtakeArmState == ARM_STATE.HIGH_CHAMBER) &&
                outtakeWristState == WRIST_STATE.DROP);
    }

    public boolean outtakeSensingActivated = true;
    public boolean outtakeSampleSensed = false;
    public ColorRange sensedSampleColor = ColorRange.GREEN;
    public double SENSE_DISTANCE = 10;
    public float[] sensedSampleHsvValues = new float[3];
    public NormalizedRGBA sensedColor;
    public double outtakeSensingDistance = 500;

    public void senseOuttakeSampleColor(){
        if (outtakeSensingActivated) {
            if (outtakeSensor instanceof DistanceSensor){
                outtakeSensingDistance = ((DistanceSensor) outtakeSensor).getDistance(DistanceUnit.MM);
            }
            if(outtakeSensingDistance < SENSE_DISTANCE){
                outtakeSampleSensed = true;
                sensedColor = outtakeSensor.getNormalizedColors();
                Color.colorToHSV(sensedColor.toColor(), sensedSampleHsvValues);
            } else {
                outtakeSampleSensed = false;
            }

        } else {
            outtakeSampleSensed = false;
            sensedSampleColor = ColorRange.GREEN;
        }
    }



    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Outtake Slides");
        telemetry.addData("    State", outtakeSlidesState);
        telemetry.addData("    Last State", lastOuttakeSlideState);
        telemetry.addData("    isOuttakeSlidesInTransfer", isOuttakeSlidesInState(SLIDE_STATE.TRANSFER));
        telemetry.addData("    outtakeStallTimingFlag", outtakeStallTimingFlag);
        telemetry.addData("    outtakeStallTimer.time", outtakeStallTimer.time());
        telemetry.addData("    outtakeTouch.getState", outtakeTouch.getState());
        telemetry.addData("    Left Motor Position", outtakeSlideLeft.getCurrentPosition());
        telemetry.addData("    Right Motor Position", outtakeSlideRight.getCurrentPosition());
        //telemetry.addData("    Left Climb Motor Position", outtakeSlideLeftClimb.getCurrentPosition());
        //telemetry.addData("    Right Climb Motor Position", outtakeSlideRightClimb.getCurrentPosition());
        telemetry.addData("    Outtake Slides Left Current", outtakeSlideLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("    Outtake Slides Right Current", outtakeSlideRight.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("    Outtake Slides Left Climb Current", outtakeSlideLeftClimb.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("    Outtake Slides Right Climb Current", outtakeSlideRightClimb.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("=============");
        telemetry.addLine("Outtake Arm");
        telemetry.addData("   State", outtakeArmState);
        telemetry.addData("   Servo position", outtakeArmServo.getPosition());
        telemetry.addLine("Outtake Wrist");
        telemetry.addData("   State", outtakeWristState);
        telemetry.addData("   Servo position", outtakeWristServo.getPosition());
        telemetry.addLine("Outtake Grip");
        telemetry.addData("   State", outtakeGripState);
        telemetry.addData("   Grip Servo position", outtakeGripServo.getPosition());
        telemetry.addLine("Outtake Sensor");
        telemetry.addData("   outtakeSensingActivated", outtakeSensingActivated);
        if (outtakeSensingActivated) {
            telemetry.addLine("Outtake Sensor");
            telemetry.addData("   intakeSensingDistance", outtakeSensingDistance);
            telemetry.addData("   intakeSampleSensed", outtakeSampleSensed);
            if (outtakeSampleSensed) {
                telemetry.addData("    RGB","%.3f, %.3f, %.3f", sensedColor.red, sensedColor.green, sensedColor.blue);
                telemetry.addData("    HSVA","%.3f, %.3f, %.3f, %.3f", sensedSampleHsvValues[0], sensedSampleHsvValues[1], sensedSampleHsvValues[2], sensedColor.alpha);
            }
        }

    }


}

