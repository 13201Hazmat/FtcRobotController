package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//slide movement should be on bicubic dynamic acceleration control
public class IntakeSlides {
    public Servo intakeSlideServoLeft, intakeSlideServoRight;

    //IntakeSlides servo states
    public enum INTAKE_SLIDES_STATE{
        MIN_RETRACTED (0,0),
        TRANSFER (0.1, 0.1),
        PICKUP(1.0, 1.0);

        public double leftSlidePos;
        public double rightSlidePos;
        INTAKE_SLIDES_STATE(double leftSlidePos, double rightSlidePos){
            this.leftSlidePos = leftSlidePos;
            this.rightSlidePos = rightSlidePos;
        }
    }
    public INTAKE_SLIDES_STATE intakeSlidesState = INTAKE_SLIDES_STATE.TRANSFER;

    public void setIntakeSlidesState(INTAKE_SLIDES_STATE intakeSlidesState, int slideExtension) {
        intakeSlidesState.leftSlidePos = slideExtension;
        intakeSlidesState.rightSlidePos = slideExtension;
    }

    public double leftIntakeSlideCurrPos, leftIntakeSlideNewPos = intakeSlidesState.leftSlidePos;
    public double rightIntakeSlideCurrPos, rightIntakeSlideNewPos = intakeSlidesState.rightSlidePos;

    public IntakeSlides(HardwareMap hardwareMap){
        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
        initIntakeSlides();
    }

    public void initIntakeSlides(){
        intakeSlideServoLeft.setDirection(Servo.Direction.FORWARD);
        intakeSlideServoRight.setDirection(Servo.Direction.REVERSE);
        moveIntakeSlides(INTAKE_SLIDES_STATE.MIN_RETRACTED);
    }

    public void moveIntakeSlides(INTAKE_SLIDES_STATE intakeSlidesState) {
        intakeSlideServoLeft.setPosition(intakeSlidesState.leftSlidePos);
        intakeSlideServoRight.setPosition(intakeSlidesState.rightSlidePos);
        setIntakeSlidesState(intakeSlidesState, (int) intakeSlidesState.leftSlidePos);
        this.intakeSlidesState = intakeSlidesState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Intake Slides");
        telemetry.addData("    State", intakeSlidesState);
        telemetry.addData("    Left Servo Position", intakeSlideServoLeft.getPosition());
        telemetry.addData("    Right Servo Position", intakeSlideServoRight.getPosition());
        telemetry.addLine("=============");
    }

}
