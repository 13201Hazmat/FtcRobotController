package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlides {
    public Servo outtakeSlideServoLeft, outtakeSlideServoRight;

    public enum OUTTAKE_SLIDES_STATE{
        TRANSFER(0),
        LOW_CHAMBER(0.4),
        HIGH_CHAMBER(0.8);

        public double slidePos;
        OUTTAKE_SLIDES_STATE(double slidePos){
            this.slidePos = slidePos;
        }

    }
    public OuttakeSlides.OUTTAKE_SLIDES_STATE outtakeSlidesState = OUTTAKE_SLIDES_STATE.TRANSFER;

    public void setOuttakeSlide(IntakeSlides.INTAKE_SLIDES_STATE intakeSlidesState, int slideExtension) {
        intakeSlidesState.slidePos = slideExtension;
    }

    public double intakeSlidesCurrPos, intakeSlidesNewPos = intakeSlidesState.slidePos;

    public IntakeSlides(HardwareMap hardwareMap){
        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
        initIntakeSlides();
    }

    public void initIntakeSlides(){
        intakeSlideServoLeft.setDirection(Servo.Direction.FORWARD);
        intakeSlideServoRight.setDirection(Servo.Direction.REVERSE);
        moveSlides(IntakeSlides.INTAKE_SLIDES_STATE.MIN_RETRACTED);
    }

    public void moveSlides(IntakeSlides.INTAKE_SLIDES_STATE intakeSlidesState) {
        intakeSlideServoLeft.setPosition(intakeSlidesState.slidePos);
        intakeSlideServoRight.setPosition(intakeSlidesState.slidePos);
        this.intakeSlidesState = intakeSlidesState;
    }
}
