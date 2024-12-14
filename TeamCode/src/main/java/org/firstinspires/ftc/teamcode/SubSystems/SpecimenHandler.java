package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpecimenHandler {
    public Servo gripServo;
    public DcMotorEx specimenSlide;

    public enum GRIP_STATE {
        OPEN(0.1),
        CLOSED(0.28);

        private final double gripPosition;

        GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }

    public GRIP_STATE gripState = GRIP_STATE.CLOSED;

    public boolean autoOpenSpecimenGrip = true;

    public static double CONVERT_435_TO_1150 = 435.0/1150.0 ;

    //Outtake Motor states
    public enum SLIDE_STATE {
        MIN_RETRACTED_LOW_CHAMBER_LATCH(0),
        PICKUP(100), //100 for 435 rpm
        LOW_CHAMBER(400), //400 for 435 rpm
        HICH_CHAMBER_LATCH(1050), //1100 for 435 rpm
        HIGH_CHAMBER(1500), //1500 for 435 rpm
        MAX_EXTENDED(2280); //2280 for 435 rpm

        public final double motorPosition;

        SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }
    }

    public SLIDE_STATE specimenSlidesState = SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH;

    public int specimenMotorCurrentPosition = 0;
    public double specimenMotorNewPosition = specimenSlidesState.motorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 100;//200

    //Different constants of arm speed
    public static final double SPECIMEN_MOTOR_POWER = 1.0;//0.75


    public boolean runOuttakeMotorToLevelState = false;

    Telemetry telemetry;

    public SpecimenHandler(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        gripServo = hardwareMap.get(Servo.class, "specimen_grip");
        specimenSlide = hardwareMap.get(DcMotorEx.class, "specimen_slide");
        initSpecimenHandler();
    }

    public void initSpecimenHandler() {

        closeGrip();

        resetOuttakeMotorMode();
        specimenSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        specimenSlide.setPositionPIDFCoefficients(5.0);

        turnOuttakeBrakeModeOff();
        specimenSlidesState = SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH;
    }

    /**
     * If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip() {
        gripServo.setPosition(GRIP_STATE.OPEN.gripPosition);
        gripState = GRIP_STATE.OPEN;
    }

    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip() {
        gripServo.setPosition(GRIP_STATE.CLOSED.gripPosition);
        gripState = GRIP_STATE.CLOSED;
    }

    public void toggleGrip() {
        if(gripState == GRIP_STATE.OPEN) {
            closeGrip();
        } else {
            openGrip();
        }
    }


    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        specimenSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        specimenSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets outtake slides to Transfer position
    public void moveSpecimenSlides(SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        specimenMotorCurrentPosition = specimenSlide.getCurrentPosition();
        specimenSlide.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        specimenSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    public void lowerSlideToLatch(){
        if (specimenSlidesState == SLIDE_STATE.HIGH_CHAMBER) {
            moveSpecimenSlides(SLIDE_STATE.HICH_CHAMBER_LATCH);
        }
        if (specimenSlidesState == SLIDE_STATE.LOW_CHAMBER) {
            moveSpecimenSlides(SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH);
        }
    }

    public void backToInit(){
        moveSpecimenSlides(SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH);
        closeGrip();
    }

    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (specimenSlidesState == SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH) {
            turnOuttakeBrakeModeOff();
        } else {
            turnOuttakeBrakeModeOn();
        }
        power = SPECIMEN_MOTOR_POWER;

        specimenSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runOuttakeMotorToLevelState == true){
            specimenSlide.setPower(power);
            runOuttakeMotorToLevelState = false;
        } else{
            specimenSlide.setPower(0.0);
        }
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode1 = specimenSlide.getMode();
        DcMotorSimple.Direction directiom = specimenSlide.getDirection();
        specimenSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        specimenSlide.setMode(runMode1);
        specimenSlide.setPositionPIDFCoefficients(5.0);
        specimenSlide.setDirection(directiom);
    }

    //TODO : Add logic to use Voltage Sensor to measure motor stalling and reset.
    public void manualResetSpecimenHandlerMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        specimenSlide.setTargetPosition((int) (specimenSlide.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        specimenSlidesState = SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(SLIDE_STATE toOuttakeSlideState) {
        isOuttakeSlidesInStateError = Math.abs(specimenSlide.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        return (specimenSlidesState == toOuttakeSlideState && (isOuttakeSlidesInStateError <= 30));
    }

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Specimen Handler");
        telemetry.addData("    State", specimenSlidesState);
        telemetry.addData("    Specimen Slide Position", specimenSlide.getCurrentPosition());
        telemetry.addLine("=============");
        telemetry.addLine("Specimen Grip");
        telemetry.addData("    State", gripState);
        telemetry.addData("    Grip Servo position", gripServo.getPosition());
        telemetry.addData("    AutoGripOpen", autoOpenSpecimenGrip);
        telemetry.addLine("=============");
    }


}

