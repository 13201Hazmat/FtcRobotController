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


    public enum ARM_STATE {
        //Calib Position : Fully in mechanical limit inwards is One
        INIT(1.0),
        TRANSFER(1.0),
        DROP(0.42),
        EJECT(0.42),
        MAX(0.0);

        private double armPos;
        ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public ARM_STATE outtakeArmState = ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public enum WRIST_STATE {
        //Calib Position : Fully in mechanical limit inwards is One
        INIT(0.55),
        TRANSFER(0.55),
        PRE_DROP(0.73),
        EJECT(0.96),
        DROP(0.96),
        MAX(0.60);

        private double wristPos;
        WRIST_STATE(double wristPos){
            this.wristPos = wristPos;
        }
    }
    public WRIST_STATE outtakeWristState = WRIST_STATE.TRANSFER;
    public double WRIST_DELTA = 0.01;

    //Outtake Motor states
    public enum SLIDE_STATE {
        MIN_RETRACTED(0, 0),
        EJECT(0,0),
        TRANSFER(0, 0),
        LOW_BUCKET(600, 600),
        HIGH_BUCKET(2100, 2100),
        CLIMBER2(2000, 2000),
        MAX_EXTENDED(2280, 2280);

        public final double leftMotorPosition;
        public final double rightMotorPosition;
        SLIDE_STATE(double leftMotorPosition, double rightMotorPosition) {
            this.leftMotorPosition = leftMotorPosition;
            this.rightMotorPosition = rightMotorPosition;
        }
    }

    public SLIDE_STATE outtakeSlidesState = SLIDE_STATE.TRANSFER;

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
        moveArm(ARM_STATE.TRANSFER);
        resetOuttakeMotorMode();
        outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setPositionPIDFCoefficients(5.0);
        outtakeSlideRight.setPositionPIDFCoefficients(5.0);
        outtakeSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotorEx.Direction.FORWARD);
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = SLIDE_STATE.TRANSFER;
    }

    public void moveArm(ARM_STATE toOuttakeArmState){
        outtakeArmServo.setPosition(toOuttakeArmState.armPos);
        moveWrist(toOuttakeArmState);
        outtakeArmState = toOuttakeArmState;
    }

    public void moveWrist(ARM_STATE outtakeArmState){
        switch (outtakeArmState){
            case INIT:
                outtakeWristServo.setPosition(WRIST_STATE.INIT.wristPos);
                outtakeWristState = WRIST_STATE.INIT;
                break;
            case TRANSFER:
                outtakeWristServo.setPosition(WRIST_STATE.TRANSFER.wristPos);
                outtakeWristState = WRIST_STATE.TRANSFER;
                break;
            case DROP:
                outtakeWristServo.setPosition(WRIST_STATE.PRE_DROP.wristPos);
                outtakeWristState = WRIST_STATE.PRE_DROP;
                break;
            case EJECT:
                outtakeWristServo.setPosition(WRIST_STATE.EJECT.wristPos);
                outtakeWristState = WRIST_STATE.EJECT;
                break;
            case MAX:
                outtakeWristServo.setPosition(WRIST_STATE.MAX.wristPos);
                outtakeWristState = WRIST_STATE.MAX;
                break;
        }
    }

    public void moveWristDrop(){
        if (outtakeWristState == WRIST_STATE.PRE_DROP) {
            outtakeWristServo.setPosition(WRIST_STATE.DROP.wristPos);
            outtakeWristState = WRIST_STATE.DROP;
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
    public void moveOuttakeSlides(SLIDE_STATE toOuttakeMotorState){
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
        if (outtakeSlidesState == SLIDE_STATE.TRANSFER) {
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
        outtakeSlideLeft.setTargetPosition((int) (outtakeSlideLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        outtakeSlideRight.setTargetPosition((int) (outtakeSlideRight.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = SLIDE_STATE.MIN_RETRACTED;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(SLIDE_STATE toOuttakeSlideState) {
        //isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesInStateError = Math.abs(outtakeSlideLeft.getCurrentPosition() - toOuttakeSlideState.leftMotorPosition);
        return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 100);
    }

    public boolean isOuttakeInTransfer(){
        return (isOuttakeSlidesInState(SLIDE_STATE.TRANSFER) &&
                        outtakeArmState == ARM_STATE.TRANSFER &&
                        outtakeWristState == WRIST_STATE.TRANSFER);
    }

    public boolean isOuttakeReadyToDrop(){
        return (outtakeArmState == ARM_STATE.DROP &&
                outtakeWristState == WRIST_STATE.PRE_DROP);
    }

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Outtake Slides");
        telemetry.addData("    State", outtakeSlidesState);
        telemetry.addData("    isOuttakeSlidesInTransfer", isOuttakeSlidesInState(SLIDE_STATE.TRANSFER));
        telemetry.addData("    Left Motor Position", outtakeSlideLeft.getCurrentPosition());
        telemetry.addData("    Right Motor Position", outtakeSlideRight.getCurrentPosition());
        telemetry.addLine("=============");
        telemetry.addLine("Outtake Arm");
        telemetry.addData("   State", outtakeArmState);
        telemetry.addData("   Servo position", outtakeArmServo.getPosition());
        telemetry.addLine("=============");
        telemetry.addLine("Outtake Wrist");
        telemetry.addData("   State", outtakeWristState);
        telemetry.addData("   Servo position", outtakeWristServo.getPosition());
        telemetry.addLine("=============");

    }


}

