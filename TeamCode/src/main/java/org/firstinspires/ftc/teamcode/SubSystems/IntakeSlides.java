package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

//slide movement should be on bicubic dynamic acceleration control
public class IntakeSlides {
    public Servo intakeSlideServoLeft, intakeSlideServoRight;

    //Intake servo states
    public enum INTAKE_SLIDES_STATE{
        MIN_RETRACTED (0),
        TRANSFER (0),
        MAX_EXTENDED (500);

        public double slidePos;
        INTAKE_SLIDES_STATE(double slidePos){
            this.slidePos = slidePos;
        }
    }

    public INTAKE_SLIDES_STATE intakeSlidesState = INTAKE_SLIDES_STATE.TRANSFER;

    public void setIntakeSlide(INTAKE_SLIDES_STATE intakeSlidesState, int slideExtension) {
        intakeSlidesState.slidePos = slideExtension;
    }

    public double intakeSlidesCurrPos, intakeSlidesNewPos = intakeSlidesState.slidePos;

    public IntakeSlides(HardwareMap hardwareMap){
        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
        initIntakeSlides();
    }

    public void initIntakeSlides(){
        moveSlides(INTAKE_SLIDES_STATE.MIN_RETRACTED);
    }

    public void moveSlides(INTAKE_SLIDES_STATE toSlidesState) {
        intakeSlideServoLeft.setPosition(toSlidesState.slidePos);
        intakeSlideServoRight.setPosition(toSlidesState.slidePos);
        intakeSlidesState = toSlidesState;
    }


}
