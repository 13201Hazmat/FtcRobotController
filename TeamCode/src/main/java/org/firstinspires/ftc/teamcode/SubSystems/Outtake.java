package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public Servo leftPTOServo;
    public Servo rightPTOServo;
    public NormalizedColorSensor outtakeSensor;
    public DcMotorEx outtakeSlideLeft;
    public DcMotorEx outtakeSlideLeftClimb;
    public DcMotorEx outtakeSlideRight;
    public DcMotorEx outtakeSlideRightClimb;
    public DigitalChannel outtakeTouch;

    public enum GRIP_STATE {
        OPEN(0.41), //0.52 max
        OPEN_WIDE(0.57),
        CLOSED(0.17);

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
        PRE_TRANSFER(0.40),
        TRANSFER(0.37),
        AUTO_PRE_DROP(0.54),
        DROP(0.54),//0.66
        SPECIMEN_PICKUP(0.13),//0.1
        SPECIMEN_MAKE_DROP(0.44),
        HIGH_CHAMBER(0.38),//0.66
        HIGH_CHAMBER_LATCH(0.38),
        AUTO_PARK(0.33),
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
        INIT(0.3),//0.2
        PRE_TRANSFER(0.01),//0.21
        TRANSFER(0.01),//0.2
        AUTO_PRE_DROP(0.46),//0.68
        SPECIMEN_PICKUP(0.19),//0.2
        SPECIMEN_MAKE_DROP(0.75),
        HIGH_CHAMBER(0.615),//0.92
        HIGH_CHAMBER_LATCH(0.5),
        AUTO_PARK(0.27),
        DROP(0.58),//0.72
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
        LOW_BUCKET(200),
        SPECIMEN_PICKUP(0),
        SPECIMEN_MAKE_DROP(0),
        HIGH_BUCKET(1300),
        HIGHER_BUCKET(1400),
        HIGH_CHAMBER(0),
        HIGH_CHAMBER_LATCH(200),
        LEVEL2_ASCEND(1900),//700 for lower bar
        LEVEL2_CLIMB_ENGAGED(1750), //600 for lower bar
        LEVEL2_CLIMB(1200), //0 for lower bar
        LEVEL3_ASCEND(700),//700 for lower bar
        LEVEL3_CLIMB_ENGAGED(600), //600 for lower bar
        LEVEL3_CLIMB(300), //0 for lower bar
        MAX_EXTENDED(2000);

        public final double motorPosition;
        SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;

        }
    }
    public IntakeSlides.SLIDES_STATE intakeSlidesState = IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED;

    public SLIDE_STATE outtakeSlidesState = SLIDE_STATE.TRANSFER;
    public SLIDE_STATE lastOuttakeSlideState = SLIDE_STATE.TRANSFER;

    public enum PTO_STATE {
        PTO_OFF(0.67, 0.47),
        PTO_ON(0.47,0.27);

        public double leftPTOPos;
        public double rightPTOPos;
        PTO_STATE(double leftPTOPos, double rightPTOPos){
            this.leftPTOPos = leftPTOPos;
            this.rightPTOPos = rightPTOPos;
        }
    }
    public PTO_STATE ptoState = PTO_STATE.PTO_OFF;

    public boolean climberAscended = false;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 50;//200

    //Different constants of arm speed
    public double outtakeMotorPower = 1.0;//0.75

    public ElapsedTime outtakeStallTimer = new ElapsedTime(MILLISECONDS);
    public boolean outtakeStallTimingFlag = false;

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
        outtakeSlideLeftClimb = hardwareMap.get(DcMotorEx.class, "outtake_climb_left");
        outtakeSlideRightClimb = hardwareMap.get(DcMotorEx.class, "outtake_climb_right");
        outtakeTouch = hardwareMap.get(DigitalChannel .class, "outtakeTouch");
        leftPTOServo = hardwareMap.get(Servo.class, "left_pto");
        rightPTOServo = hardwareMap.get(Servo.class, "right_pto");

        initOuttake();
    }

    public void initOuttake(){
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            moveArm(ARM_STATE.INIT);
        } else {
            if (GameField.outtakeInParkPositionInAutonomous == true) {
                moveArm(ARM_STATE.AUTO_PRE_DROP);
                GameField.outtakeInParkPositionInAutonomous = false;
            } else {
                moveArm(ARM_STATE.PRE_TRANSFER);
            }
        }
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
            openGrip();
            outtakeMotorPower = 1.0;
        } else {
            outtakeMotorPower = 1.0; //0.9;
        }
        turnOuttakeClimbBrakeModeOn();
        movePTO(PTO_STATE.PTO_OFF);
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
            case PRE_TRANSFER:
                outtakeWristServo.setPosition(WRIST_STATE.PRE_TRANSFER.wristPos);
                outtakeWristState = WRIST_STATE.PRE_TRANSFER;
                openGrip();
                break;
            case TRANSFER:
                outtakeWristServo.setPosition(WRIST_STATE.TRANSFER.wristPos);
                outtakeWristState = WRIST_STATE.TRANSFER;
                openGrip();
                break;
            case AUTO_PRE_DROP:
                outtakeWristServo.setPosition(WRIST_STATE.AUTO_PRE_DROP.wristPos);
                outtakeWristState = WRIST_STATE.AUTO_PRE_DROP;
                break;
            case DROP:
                outtakeWristServo.setPosition(WRIST_STATE.DROP.wristPos);
                outtakeWristState = WRIST_STATE.DROP;
                break;
            case SPECIMEN_PICKUP:
                outtakeWristServo.setPosition(WRIST_STATE.SPECIMEN_PICKUP.wristPos);
                outtakeWristState = WRIST_STATE.SPECIMEN_PICKUP;
                break;
            case SPECIMEN_MAKE_DROP:
                outtakeWristServo.setPosition(WRIST_STATE.SPECIMEN_MAKE_DROP.wristPos);
                outtakeWristState = WRIST_STATE.SPECIMEN_MAKE_DROP;
                break;
            case HIGH_CHAMBER:
                outtakeWristServo.setPosition(WRIST_STATE.HIGH_CHAMBER.wristPos);
                outtakeWristState = WRIST_STATE.HIGH_CHAMBER;
                break;
            case HIGH_CHAMBER_LATCH:
                outtakeWristServo.setPosition(WRIST_STATE.HIGH_CHAMBER_LATCH.wristPos);
                outtakeWristState = WRIST_STATE.HIGH_CHAMBER;
                break;
            case AUTO_PARK:
                outtakeWristServo.setPosition(WRIST_STATE.AUTO_PARK.wristPos);
                outtakeWristState = WRIST_STATE.AUTO_PARK;
                break;
            case MAX:
                outtakeWristServo.setPosition(WRIST_STATE.MAX.wristPos);
                outtakeWristState = WRIST_STATE.MAX;
                break;
        }
        //adjustOpenGrip();
    }

    public void ascendToClimbLevel2(){
        movePTO(PTO_STATE.PTO_OFF);
        moveOuttakeSlides(SLIDE_STATE.LEVEL2_ASCEND);
        climberAscended = true;
    }

    public void ascendToClimbLevel3(){
        movePTO(PTO_STATE.PTO_OFF);
        moveOuttakeSlides(SLIDE_STATE.LEVEL3_ASCEND);
        climberAscended = true;
    }

    public void movePTO(PTO_STATE toPTOState) {
        leftPTOServo.setPosition(toPTOState.leftPTOPos);
        rightPTOServo.setPosition(toPTOState.rightPTOPos);
        this.ptoState = toPTOState;
    }

    public void climbLevel2(){
        outtakeMotorPower = 117.0/312.0; //1.0;
        moveOuttakeSlides(SLIDE_STATE.LEVEL2_CLIMB);
        movePTO(PTO_STATE.PTO_ON);
        safeWaitMilliSeconds(100);
        startOuttakeClimbMotors();
        while(!isOuttakeSlidesInState(SLIDE_STATE.LEVEL2_CLIMB)) {
            if ( outtakeSlideLeft.getCurrentPosition() < SLIDE_STATE.LEVEL2_CLIMB_ENGAGED.motorPosition) {
                printDebugMessages();
            }
        }
        stopOuttakeClimbMotors();
    }

    public void climbLevel3Part1(){
        outtakeMotorPower = 117.0/312.0; //1.0;
        moveOuttakeSlides(SLIDE_STATE.LEVEL3_CLIMB);
        movePTO(PTO_STATE.PTO_ON);
        safeWaitMilliSeconds(100);
        startOuttakeClimbMotors();
        while(!isOuttakeSlidesInState(SLIDE_STATE.LEVEL3_CLIMB)) {
            if ( outtakeSlideLeft.getCurrentPosition() < SLIDE_STATE.LEVEL2_CLIMB_ENGAGED.motorPosition) {
                printDebugMessages();
            }
        }
        stopOuttakeClimbMotors();
    }

    public void climbLevel3Part2(){
        movePTO(PTO_STATE.PTO_OFF);
        safeWaitMilliSeconds(100);
        startOuttakeClimbMotors();
        safeWaitMilliSeconds(1000);
        reverseOuttakeClimbMotors();
        safeWaitMilliSeconds(8000);
        stopOuttakeClimbMotors();
    }



    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while ( timer.time() < time) {
        }
    }

    public void moveWristForward(){
        outtakeWristServo.setPosition(outtakeWristServo.getPosition() + WRIST_DELTA);
    }

    public void moveWristBackward(){
        outtakeWristServo.setPosition(outtakeWristServo.getPosition() - WRIST_DELTA);
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        if (outtakeArmState != ARM_STATE.SPECIMEN_PICKUP) {
            outtakeGripServo.setPosition(Outtake.GRIP_STATE.OPEN.gripPosition);
        } else {
            outtakeGripServo.setPosition(Outtake.GRIP_STATE.OPEN_WIDE.gripPosition);
        }
        outtakeGripState = Outtake.GRIP_STATE.OPEN;
    }

    public void adjustOpenGrip(){
        if (outtakeGripState == Outtake.GRIP_STATE.OPEN) {
            if (outtakeArmState != ARM_STATE.SPECIMEN_PICKUP) {
                outtakeGripServo.setPosition(Outtake.GRIP_STATE.OPEN.gripPosition);
            } else {
                outtakeGripServo.setPosition(Outtake.GRIP_STATE.OPEN_WIDE.gripPosition);
            }
        }
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
    }


    public void turnOuttakeClimbBrakeModeOn(){
        outtakeSlideLeftClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeSlideRightClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnOuttakeClimbBrakeModeOff(){
        outtakeSlideLeftClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeSlideRightClimb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }


    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        outtakeSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);    }

    public int outtakeMotorDirection = 1;
    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        /*if (toOuttakeMotorState.motorPosition - outtakeSlideLeft.getTargetPosition() >= 0) {
            outtakeMotorDirection = 1;
        } else {
            outtakeMotorDirection = -1;
        }*/
        outtakeSlideLeft.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlideRight.setTargetPosition((int)toOuttakeMotorState.motorPosition);
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
        power = outtakeMotorPower;

        outtakeSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runOuttakeMotorToLevelState == true){
            outtakeSlideLeft.setPower(power);
            outtakeSlideRight.setPower(power);
            runOuttakeMotorToLevelState = false;
        } else{
            stopOuttakeMotors();
        }
    }

    public void moveOuttakeSlidesToTransfer() {
        if (outtakeSlideLeft.getCurrentPosition() < 5 || outtakeSlideRight.getCurrentPosition() < 5) {
            resetOuttakeMotorMode();
        } else {
            moveOuttakeSlides(SLIDE_STATE.TRANSFER);
        }
    }
    
    public void stopOuttakeMotors(){
        outtakeSlideLeft.setPower(0.0);
        outtakeSlideRight.setPower(0.0);
        stopOuttakeClimbMotors();
    }

    public void startOuttakeClimbMotors(){
        outtakeSlideLeftClimb.setPower(1.0);
        outtakeSlideRightClimb.setPower(-1.0);
    }

    public void reverseOuttakeClimbMotors(){
        outtakeSlideLeftClimb.setPower(-1.0);
        outtakeSlideRightClimb.setPower(1.0);
    }

    public void stopOuttakeClimbMotors(){
        turnOuttakeBrakeModeOn();
        turnOuttakeClimbBrakeModeOn();
        outtakeSlideLeftClimb.setPower(0.0);
        outtakeSlideRightClimb.setPower(0.0);
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runModeOuttakeSlideLeft = DcMotorEx.RunMode.RUN_USING_ENCODER;
        DcMotorEx.RunMode runModeOuttakeSlideRight = DcMotorEx.RunMode.RUN_USING_ENCODER;
        outtakeSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(runModeOuttakeSlideLeft);
        outtakeSlideRight.setMode(runModeOuttakeSlideRight);
        outtakeSlideLeft.setPositionPIDFCoefficients(10.0);
        outtakeSlideRight.setPositionPIDFCoefficients(10.0);
        outtakeSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    //TODO : Add logic to use Voltage Sensor to measure motor stalling and reset.
    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        outtakeSlidesState = SLIDE_STATE.TRANSFER;
        outtakeSlideLeft.setTargetPosition((int) (outtakeSlideLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        outtakeSlideRight.setTargetPosition((int) (outtakeSlideRight.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = SLIDE_STATE.TRANSFER;
    }

    public void manualResetOuttakeMotor1() {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (timer.time() < 1200 && outtakeTouch.getState()) {
            outtakeSlideLeft.setPower(-0.7);
            outtakeSlideRight.setPower(-0.7);
        }
        stopOuttakeMotors();
        resetOuttakeMotorMode();
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(SLIDE_STATE toOuttakeSlideState) {
        if (toOuttakeSlideState == SLIDE_STATE.TRANSFER /*|| toOuttakeSlideState == SLIDE_STATE.LEVEL2_CLIMB*/) {
            return ((/*outtakeTouch.getState() == false ||*/
                    (outtakeSlideLeft.getCurrentPosition() < 5 || outtakeSlideRight.getCurrentPosition() < 5)));
        } else {
            isOuttakeSlidesInStateError = Math.abs(outtakeSlideLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
            return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 15);
        }
    }

    public void safetyReset(){
        if (outtakeSlidesState == Outtake.SLIDE_STATE.TRANSFER) {
            if ((outtakeSlideLeft.getCurrentPosition() < 5 || outtakeSlideRight.getCurrentPosition() < 5)) {//PRESSED
                stopOuttakeMotors();
                resetOuttakeMotorMode();
            }
        }
    }

    public boolean isOuttakeInPreTransfer(){
        return (isOuttakeSlidesInState(SLIDE_STATE.TRANSFER) &&
                        outtakeArmState == ARM_STATE.PRE_TRANSFER &&
                        outtakeWristState == WRIST_STATE.PRE_TRANSFER);
    }

    public boolean isOuttakeReadyToDrop(){
        return ((outtakeArmState == ARM_STATE.DROP || outtakeArmState == ARM_STATE.HIGH_CHAMBER) &&
                outtakeWristState == WRIST_STATE.DROP && outtakeGripState == GRIP_STATE.CLOSED);
    }

    public boolean outtakeSensingActivated = true;
    public boolean outtakeSampleSensed = false;
    public ColorRange sensedSampleColor = ColorRange.GREEN;
    public double SENSE_DISTANCE = 20;
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
                //TODO: IMPLEMENT COLOR SELECTION
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
        //telemetry.addData("    outtakeTouch.getState", outtakeTouch.getState());
        telemetry.addData("    Left Motor Position", outtakeSlideLeft.getCurrentPosition());
        telemetry.addData("    Right Motor Position", outtakeSlideRight.getCurrentPosition());
        telemetry.addData("    Left Motor isBusy", outtakeSlideLeft.isBusy());
        telemetry.addData("    Right Motor isBusy", outtakeSlideRight.isBusy());
        telemetry.addData("    Left Climb Motor isBusy", outtakeSlideLeftClimb.isBusy());
        telemetry.addData("    Right Climb Motor isBusy", outtakeSlideRightClimb.isBusy());
        telemetry.addData("    Outtake Slides Left Current", outtakeSlideLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("    Outtake Slides Right Current", outtakeSlideRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("    Outtake Slides Left Climb Current", outtakeSlideLeftClimb.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("    Outtake Slides Right Climb Current", outtakeSlideRightClimb.getCurrent(CurrentUnit.AMPS));

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
        telemetry.addLine("Outtake PTO");
        telemetry.addData("   State", ptoState);
        telemetry.addData("   Left PTO Servo position", leftPTOServo.getPosition());
        telemetry.addData("   Right PTO Servo position", rightPTOServo.getPosition());

    }


}

