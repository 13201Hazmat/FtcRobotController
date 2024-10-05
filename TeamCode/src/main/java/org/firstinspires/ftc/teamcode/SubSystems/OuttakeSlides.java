package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlides {
    public Servo outtakeSlideServoLeft, outtakeSlideServoRight;

    public enum OUTTAKE_SLIDES_STATE{
        TRANSFER(0.05, -0.05),
        LOW_CHAMBER(0.4, -0.4),
        HIGH_CHAMBER(0.8, -0.8);

        public double leftSlidePos;
        public double rightSlidePos;
        OUTTAKE_SLIDES_STATE(double leftSlidePos, double rightSlidePos){
            this.leftSlidePos = leftSlidePos;
            this.rightSlidePos = rightSlidePos;
        }

    }
    public OuttakeSlides.OUTTAKE_SLIDES_STATE outtakeSlidesState = OUTTAKE_SLIDES_STATE.TRANSFER;

    public void setOuttakeSlidesState(OuttakeSlides.OUTTAKE_SLIDES_STATE outtakeSlidesState, int slideExtension) {
        outtakeSlidesState.leftSlidePos = slideExtension;
        outtakeSlidesState.rightSlidePos = -slideExtension;
    }

    public double leftOuttakeSlideCurrPos, leftOuttakeSlideNewPos = outtakeSlidesState.leftSlidePos;
    public double rightOuttakeSlideCurrPos, rightOuttakeSlideNewPos = outtakeSlidesState.rightSlidePos;

    public OuttakeSlides(HardwareMap hardwareMap){
        outtakeSlideServoLeft = hardwareMap.get(Servo.class, "outtake_slide_left");
        outtakeSlideServoRight = hardwareMap.get(Servo.class, "outtake_slide_right");
        initOuttakeSlides();
    }

    public void initOuttakeSlides(){
        outtakeSlideServoLeft.setDirection(Servo.Direction.FORWARD);
        outtakeSlideServoRight.setDirection(Servo.Direction.REVERSE);
        moveOuttakeSlides(OUTTAKE_SLIDES_STATE.TRANSFER);
    }

    public void moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDES_STATE outtakeSlidesState) {
        outtakeSlideServoLeft.setPosition(outtakeSlidesState.leftSlidePos);
        outtakeSlideServoRight.setPosition(outtakeSlidesState.rightSlidePos);
        this.outtakeSlidesState = outtakeSlidesState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Outtake Slides");
        telemetry.addData("    State", outtakeSlidesState);
        telemetry.addData("    Left Servo Position", outtakeSlideServoLeft.getPosition());
        telemetry.addData("    Right Servo Position", outtakeSlideServoRight.getPosition());
        telemetry.addLine("=============");
    }
}
