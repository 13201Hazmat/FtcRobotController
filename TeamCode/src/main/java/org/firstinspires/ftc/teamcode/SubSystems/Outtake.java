package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    public Servo outtakeArmServo;
    public Servo outtakeWristServo;
    public DcMotorEx outtakeSlideLeft;
    public DcMotorEx outtakeSlideRight;


    public enum OUTTAKE_ARM_STATE{
        //Calib Position : Fully in mechanical limit inwards is Zero
        INIT(0),
        TRANSFER(0.0),
        DROP(0.57),
        MAX(0.57);

        private double armPos;
        OUTTAKE_ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public OUTTAKE_ARM_STATE outtakeArmState = OUTTAKE_ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum OUTTAKE_WRIST_STATE{
        //Calib Position : Fully in mechanical limit inwards is One
        INIT(0.55),
        TRANSFER(0.55),
        PRE_DROP(0.42),
        DROP(0.11),
        MAX(0.75);

        private double wristPos;
        OUTTAKE_WRIST_STATE(double wristPos){
            this.wristPos = wristPos;
        }
    }
    public OUTTAKE_WRIST_STATE outtakeWristState = OUTTAKE_WRIST_STATE.TRANSFER;
    public double WRIST_DELTA = 0.01;

    //Outtake Motor states
    public enum OUTTAKE_SLIDE_STATE {
        MIN_RETRACTED(0, 0),
        TRANSFER(100, 100),
        LOW_BUCKET(700, 700),
        HIGH_BUCKET(2000, 2000),
        CLIMBER2(2000, 2000),
        MAX_EXTENDED(2280, 2280);

        public final double leftMotorPosition;
        public final double rightMotorPosition;
        OUTTAKE_SLIDE_STATE(double leftMotorPosition, double rightMotorPosition) {
            this.leftMotorPosition = leftMotorPosition;
            this.rightMotorPosition = rightMotorPosition;
        }
    }

    public OUTTAKE_SLIDE_STATE outtakeSlidesState = OUTTAKE_SLIDE_STATE.TRANSFER;

    public int outtakeMotorLeftCurrentPosition, outtakeMotorRightCurrentPosition = 0;
    public double outtakeMotorLeftNewPosition, outtakeMotorRightNewPosition = outtakeSlidesState.leftMotorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_MAX = 50;//100
    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 50;//200

    //Different constants of arm speed
    public static final double OUTTAKE_MOTOR_POWER= 1.0;//0.75
    public enum OUTTAKE_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }

    public boolean runOuttakeMotorToLevelState = false;

    Telemetry telemetry;
    public Outtake(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        outtakeArmServo = hardwareMap.get(Servo.class, "outtake_arm");
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist");
        outtakeSlideLeft = hardwareMap.get(DcMotorEx.class, "outtake_slides_left");
        outtakeSlideRight = hardwareMap.get(DcMotorEx.class, "outtake_slides_right");
        initOuttake();
    }

    public void initOuttake(){
        moveArm(OUTTAKE_ARM_STATE.TRANSFER);
        resetOuttakeMotorMode();
        outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setPositionPIDFCoefficients(10.0);
        outtakeSlideRight.setPositionPIDFCoefficients(10.0);
        outtakeSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotorEx.Direction.FORWARD);
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = OUTTAKE_SLIDE_STATE.TRANSFER;
    }

    public void moveArm(OUTTAKE_ARM_STATE outtakeArmState){
        outtakeArmServo.setPosition(outtakeArmState.armPos);
        moveWrist(outtakeArmState);
        this.outtakeArmState = outtakeArmState;
    }

    public void moveWrist(OUTTAKE_ARM_STATE outtakeArmState){
        switch (outtakeArmState){
            case INIT:
                outtakeWristServo.setPosition(OUTTAKE_WRIST_STATE.INIT.wristPos);
                break;
            case TRANSFER:
                outtakeWristServo.setPosition(OUTTAKE_WRIST_STATE.TRANSFER.wristPos);
                break;
            case DROP:
                outtakeWristServo.setPosition(OUTTAKE_WRIST_STATE.PRE_DROP.wristPos);
                break;
            case MAX:
                outtakeWristServo.setPosition(OUTTAKE_WRIST_STATE.MAX.wristPos);
                break;
        }
    }

    public void moveWristDrop(){
        if (outtakeWristState == OUTTAKE_WRIST_STATE.PRE_DROP) {
            outtakeWristState = OUTTAKE_WRIST_STATE.DROP;
            outtakeWristServo.setPosition(OUTTAKE_WRIST_STATE.DROP.wristPos);
        }
    }


    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        outtakeSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        outtakeSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(OUTTAKE_SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        outtakeMotorLeftCurrentPosition = outtakeSlideLeft.getCurrentPosition();
        outtakeMotorRightCurrentPosition = outtakeSlideRight.getCurrentPosition();
        outtakeSlideLeft.setTargetPosition((int)toOuttakeMotorState.leftMotorPosition);
        outtakeSlideRight.setTargetPosition((int)toOuttakeMotorState.leftMotorPosition);
        outtakeSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (outtakeSlidesState == OUTTAKE_SLIDE_STATE.TRANSFER) {
            turnOuttakeBrakeModeOff();
        } else {
            turnOuttakeBrakeModeOn();
        }
        power = OUTTAKE_MOTOR_POWER;

        outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runOuttakeMotorToLevelState == true){
            outtakeSlideLeft.setPower(power);
            outtakeSlideRight.setPower(power);
            runOuttakeMotorToLevelState = false;
        } else{
            outtakeSlideLeft.setPower(0.0);
            outtakeSlideRight.setPower(0.0);
        }
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode1 = outtakeSlideLeft.getMode();
        DcMotorEx.RunMode runMode2 = outtakeSlideRight.getMode();
        outtakeSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(runMode1);
        outtakeSlideRight.setMode(runMode2);
        outtakeSlideLeft.setPositionPIDFCoefficients(5.0);
        outtakeSlideRight.setPositionPIDFCoefficients(5.0); //5
        outtakeSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    //TODO : Add logic to use Voltage Sensor to measure motor stalling and reset.
    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        //while (outtakeTouch.getState() && timer.time() < 5000) { TODO : Do it without touch sensor
        outtakeSlideLeft.setTargetPosition((int) (outtakeSlideLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        outtakeSlideRight.setTargetPosition((int) (outtakeSlideRight.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        //}
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE toOuttakeSlideState) {
        //isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesInStateError = Math.abs(outtakeSlideRight.getCurrentPosition() - toOuttakeSlideState.leftMotorPosition);
        return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 30);
    }

    public boolean isOuttakeInTransfer(){
        return (isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE.TRANSFER) &&
                        outtakeArmState == OUTTAKE_ARM_STATE.TRANSFER &&
                        outtakeWristState == OUTTAKE_WRIST_STATE.TRANSFER);
    }

    public boolean isOuttakeReadyToDrop(){
        return ((isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE.LOW_BUCKET) ||
                (isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE.HIGH_BUCKET)) &&
                outtakeArmState == OUTTAKE_ARM_STATE.DROP &&
                outtakeWristState == OUTTAKE_WRIST_STATE.PRE_DROP));
    }

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Outtake Slides");
        telemetry.addData("    State", outtakeSlidesState);
        telemetry.addData("    Left Motor Position", outtakeSlideLeft.getCurrentPosition());
        telemetry.addData("    Right Motor Position", outtakeSlideRight.getCurrentPosition());
        telemetry.addLine("=============");
        telemetry.addLine("Outtake Arm");
        telemetry.addData("   State", outtakeArmState);
        telemetry.addData("   Servo position", outtakeArmServo.getPosition());
        telemetry.addLine("=============");
    }


}

