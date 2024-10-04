package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

//slide movement should be on bicubic dynamic acceleration control
public class IntakeSlides {
    public Servo intakeSlideServoLeft, intakeSlideServoRight;

    //Intake servo states
    public enum INTAKE_SLIDES_STATE{
        MIN_RETRACTED (0,0),
        TRANSFER (0.1, 0.1),
        MAX_EXTENDED (1.0, 1.0);

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

    public double intakeSlidesCurrPos, intakeSlidesNewPos = intakeSlidesState.slidePos;

    public IntakeSlides(HardwareMap hardwareMap){
        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
        initIntakeSlides();
    }

    public void initIntakeSlides(){
        intakeSlideServoLeft.setDirection(Servo.Direction.FORWARD);
        intakeSlideServoRight.setDirection(Servo.Direction.REVERSE);
        moveSlides(INTAKE_SLIDES_STATE.MIN_RETRACTED);
    }

    public void moveSlides(INTAKE_SLIDES_STATE intakeSlidesState) {
        intakeSlideServoLeft.setPosition(intakeSlidesState.slidePos);
        intakeSlideServoRight.setPosition(intakeSlidesState.slidePos);
        this.intakeSlidesState = intakeSlidesState;
    }

}
