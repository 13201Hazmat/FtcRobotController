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

    public enum SPECIMEN_GRIP_STATE {
        OPEN(0.1),
        CLOSED(0.28);

        private final double gripPosition;
        SPECIMEN_GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public SPECIMEN_GRIP_STATE gripState = SPECIMEN_GRIP_STATE.CLOSED;

    //Outtake Motor states
    public enum SPECIMEN_SLIDE_STATE {
        MIN_RETRACTED(0),
        PICKUP(100),
        LOW_CHAMBER(400),
        HIGH_CHAMBER(1500),
        MAX_EXTENDED(2280);

        public final double motorPosition;
        SPECIMEN_SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }
    }

    public SPECIMEN_SLIDE_STATE specimenSlidesState = SPECIMEN_SLIDE_STATE.PICKUP;

    public int SLIDE_LOWER_DELTA_TO_LATCH = 400;

    public int specimenMotorCurrentPosition = 0;
    public double specimenMotorNewPosition = specimenSlidesState.motorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_MAX = 50;//100
    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 50;//200

    //Different constants of arm speed
    public static final double SPECIMEN_MOTOR_POWER= 1.0;//0.75
    public enum SPECIMEN_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }

    public boolean runOuttakeMotorToLevelState = false;

    Telemetry telemetry;
    public SpecimenHandler(HardwareMap hardwareMap, Telemetry telemetry) { //map hand servo's to each
        this.telemetry = telemetry;
        gripServo = hardwareMap.get(Servo.class, "specimen_grip");
        specimenSlide = hardwareMap.get(DcMotorEx.class, "specimen_slide");
        initSpecimenHandler();
    }

    public void initSpecimenHandler(){

        closeGrip();

        resetOuttakeMotorMode();
        specimenSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        specimenSlide.setPositionPIDFCoefficients(10.0);

        turnOuttakeBrakeModeOff();
        specimenSlidesState = SPECIMEN_SLIDE_STATE.PICKUP;
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        gripServo.setPosition(SPECIMEN_GRIP_STATE.OPEN.gripPosition);
        gripState = SPECIMEN_GRIP_STATE.OPEN;
    }

    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        gripServo.setPosition(SPECIMEN_GRIP_STATE.CLOSED.gripPosition);
        gripState = SPECIMEN_GRIP_STATE.CLOSED;
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
    public void moveSpecimenSlides(SPECIMEN_SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        specimenMotorCurrentPosition = specimenSlide.getCurrentPosition();
        specimenSlide.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        specimenSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    public void lowerSlideToLatch(){
         specimenSlide.setTargetPosition((int)(specimenSlidesState.motorPosition - SLIDE_LOWER_DELTA_TO_LATCH));
         runOuttakeMotorToLevelState = true;
         runOuttakeMotorToLevel();
    }

    public void backToInit(){
        moveSpecimenSlides(SPECIMEN_SLIDE_STATE.MIN_RETRACTED);
        closeGrip();
    }

    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (specimenSlidesState == SPECIMEN_SLIDE_STATE.PICKUP) {
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
        specimenSlidesState = SPECIMEN_SLIDE_STATE.MIN_RETRACTED;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(SPECIMEN_SLIDE_STATE toOuttakeSlideState) {
        isOuttakeSlidesInStateError = Math.abs(specimenSlide.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        return (specimenSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 30);
    }

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addLine("Specimen Handler");
        telemetry.addData("    State", specimenSlidesState);
        telemetry.addData("    Specimen Slide Position", specimenSlide.getCurrentPosition());
        telemetry.addLine("=============");
        telemetry.addLine("Specimen Grip");
        telemetry.addData("    State", gripServo);
        telemetry.addData("    Grip Servo position", gripServo.getPosition());
        telemetry.addLine("=============");
    }


}

