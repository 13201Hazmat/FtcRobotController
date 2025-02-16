package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//slide movement should be on bicubic dynamic acceleration control
public class IntakeSlides {
    public Servo intakeSlideServoLeft, intakeSlideServoRight;

    //IntakeSlides servo states
    public enum SLIDES_STATE {
        TRANSFER_MIN_RETRACTED (0.20, 0.245),
        IN_BETWEEN(0.35,0.395),
        MAX_EXTENSION(0.575,0.620); //0.450,0.495

        public double leftSlidePos;
        public double rightSlidePos;
        SLIDES_STATE(double leftSlidePos, double rightSlidePos){
            this.leftSlidePos = leftSlidePos;
            this.rightSlidePos = rightSlidePos;
        }
    }
    public SLIDES_STATE intakeSlidesState = SLIDES_STATE.TRANSFER_MIN_RETRACTED;

    public double INTAKE_SLIDE_DELTA = 0.1;//0.01 TODO: CHange to 0.01 if caliberating Camera


    public double leftIntakeSlideCurrPos, leftIntakeSlideNewPos = intakeSlidesState.leftSlidePos;
    public double rightIntakeSlideCurrPos, rightIntakeSlideNewPos = intakeSlidesState.rightSlidePos;

    Telemetry telemetry;
    public IntakeSlides(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
        initIntakeSlides();
    }

    public void initIntakeSlides(){
        //intakeSlideServoLeft.setDirection(Servo.Direction.FORWARD);
        //intakeSlideServoRight.setDirection(Servo.Direction.REVERSE);
        moveIntakeSlides(SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        intakeSlidesState = SLIDES_STATE.TRANSFER_MIN_RETRACTED;
    }

    public void moveIntakeSlides(SLIDES_STATE intakeSlidesState) {
        intakeSlideServoLeft.setPosition(intakeSlidesState.leftSlidePos);
        intakeSlideServoRight.setPosition(intakeSlidesState.rightSlidePos);
        this.intakeSlidesState = intakeSlidesState;
    }

    public double slideExtensionFactor(){
        return ((intakeSlideServoRight.getPosition()- SLIDES_STATE.TRANSFER_MIN_RETRACTED.rightSlidePos) /
                (SLIDES_STATE.MAX_EXTENSION.rightSlidePos - SLIDES_STATE.TRANSFER_MIN_RETRACTED.rightSlidePos));
    }

    public void moveIntakeSlidesToRange(double extensionFactor) {
        double leftSlideExtensionPosition = SLIDES_STATE.TRANSFER_MIN_RETRACTED.leftSlidePos
                + extensionFactor * (SLIDES_STATE.MAX_EXTENSION.leftSlidePos - SLIDES_STATE.TRANSFER_MIN_RETRACTED.leftSlidePos);
        double rightSlideExtensionPosition = SLIDES_STATE.TRANSFER_MIN_RETRACTED.rightSlidePos
                + extensionFactor * (SLIDES_STATE.MAX_EXTENSION.rightSlidePos - SLIDES_STATE.TRANSFER_MIN_RETRACTED.rightSlidePos);

        intakeSlideServoLeft.setPosition(leftSlideExtensionPosition);
        intakeSlideServoRight.setPosition(rightSlideExtensionPosition);
        intakeSlidesState = SLIDES_STATE.IN_BETWEEN;
    }

    public void moveIntakeSlidesSpecific(double extension){
        intakeSlideServoLeft.setPosition(extension);
        intakeSlideServoRight.setPosition(extension);
        intakeSlidesState.leftSlidePos = intakeSlideServoLeft.getPosition();
        intakeSlidesState.rightSlidePos = intakeSlideServoRight.getPosition();
    }


    public void moveIntakeSlidesForward(){
        double intakeSlideServoLeftPosition = intakeSlideServoLeft.getPosition();
        double intakeSlideServoRightPosition = intakeSlideServoRight.getPosition();

        if (intakeSlideServoLeftPosition + INTAKE_SLIDE_DELTA < SLIDES_STATE.MAX_EXTENSION.leftSlidePos) {
            intakeSlideServoLeft.setPosition(intakeSlideServoLeftPosition + INTAKE_SLIDE_DELTA);
            intakeSlideServoRight.setPosition(intakeSlideServoRightPosition + INTAKE_SLIDE_DELTA);
            intakeSlidesState = SLIDES_STATE.IN_BETWEEN;
        } else if (intakeSlidesState == SLIDES_STATE.IN_BETWEEN ){
            intakeSlideServoLeft.setPosition(SLIDES_STATE.MAX_EXTENSION.leftSlidePos);
            intakeSlideServoRight.setPosition(SLIDES_STATE.MAX_EXTENSION.rightSlidePos);
            intakeSlidesState = SLIDES_STATE.MAX_EXTENSION;
        } else {
            intakeSlideServoLeft.setPosition(SLIDES_STATE.MAX_EXTENSION.leftSlidePos);
            intakeSlideServoRight.setPosition(SLIDES_STATE.MAX_EXTENSION.rightSlidePos);
            intakeSlidesState = SLIDES_STATE.MAX_EXTENSION;
        }

    }

    public void moveIntakeSlidesBackward() {
        double intakeSlideServoLeftPosition = intakeSlideServoLeft.getPosition();
        double intakeSlideServoRightPosition = intakeSlideServoRight.getPosition();

        if (intakeSlidesState == SLIDES_STATE.MAX_EXTENSION) {
            intakeSlideServoLeft.setPosition(SLIDES_STATE.MAX_EXTENSION.leftSlidePos);
            intakeSlideServoRight.setPosition(SLIDES_STATE.MAX_EXTENSION.rightSlidePos);
            intakeSlidesState = SLIDES_STATE.MAX_EXTENSION;
        } else {
            if (intakeSlideServoLeftPosition > SLIDES_STATE.TRANSFER_MIN_RETRACTED.leftSlidePos) {
                intakeSlideServoLeft.setPosition(intakeSlideServoLeftPosition - INTAKE_SLIDE_DELTA);
                intakeSlideServoRight.setPosition(intakeSlideServoRightPosition - INTAKE_SLIDE_DELTA);
                intakeSlidesState = SLIDES_STATE.IN_BETWEEN;
            } else {
                intakeSlideServoLeft.setPosition(SLIDES_STATE.TRANSFER_MIN_RETRACTED.leftSlidePos);
                intakeSlideServoRight.setPosition(SLIDES_STATE.TRANSFER_MIN_RETRACTED.rightSlidePos);
                intakeSlidesState = SLIDES_STATE.TRANSFER_MIN_RETRACTED;
            }
        }

    }

    public void moveIntakeSlideLeftForward(){
        intakeSlideServoLeft.setPosition(intakeSlideServoLeft.getPosition() + INTAKE_SLIDE_DELTA);
    }

    public void moveIntakeSlideLeftBackward(){
        intakeSlideServoLeft.setPosition(intakeSlideServoLeft.getPosition() - INTAKE_SLIDE_DELTA);
    }

    public void moveIntakeSlideRightForward(){
        intakeSlideServoRight.setPosition(intakeSlideServoRight.getPosition() + INTAKE_SLIDE_DELTA);
    }

    public void moveIntakeSlideRightBackward(){
        intakeSlideServoRight.setPosition(intakeSlideServoRight.getPosition() - INTAKE_SLIDE_DELTA);
    }


    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Intake Slides");
        telemetry.addData("    State", intakeSlidesState);
        telemetry.addData("    Left Servo Position", intakeSlideServoLeft.getPosition());
        telemetry.addData("    Right Servo Position", intakeSlideServoRight.getPosition());
        telemetry.addData("    Extension Factor", slideExtensionFactor());
        telemetry.addLine("=============");
    }

}
