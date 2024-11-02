package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

//slide movement should be on bicubic dynamic acceleration control
public class IntakeSlides {
    public Servo intakeSlideServoLeft, intakeSlideServoRight;

    //IntakeSlides servo states
    public enum INTAKE_SLIDES_STATE{
        // Calib position - Max_Extended Left at 1.0, right at 0.0
        MIN_RETRACTED (0.81,0.19),
        TRANSFER (0.1, 0.1),
        IN_BETWEEN(0.5,0.5),
        MAX_EXTENSION(1.0, 0.0);

        public double leftSlidePos;
        public double rightSlidePos;
        INTAKE_SLIDES_STATE(double leftSlidePos, double rightSlidePos){
            this.leftSlidePos = leftSlidePos;
            this.rightSlidePos = rightSlidePos;
        }
    }
    public INTAKE_SLIDES_STATE intakeSlidesState = INTAKE_SLIDES_STATE.TRANSFER;

    public double INTAKE_SLIDE_DELTA = 0.01;


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
        intakeSlideServoLeft.setDirection(Servo.Direction.FORWARD);
        intakeSlideServoRight.setDirection(Servo.Direction.REVERSE);
        moveIntakeSlides(INTAKE_SLIDES_STATE.MIN_RETRACTED);
    }

    public void moveIntakeSlides(INTAKE_SLIDES_STATE intakeSlidesState) {
        intakeSlideServoLeft.setPosition(intakeSlidesState.leftSlidePos);
        intakeSlideServoRight.setPosition(intakeSlidesState.rightSlidePos);
        this.intakeSlidesState = intakeSlidesState;
    }

    /*
    public void locateAndMoveToBlock(ColorRange targetColor) {
        final double pixelToExtensionScale = 0.05;
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(targetColor)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        if (!blobs.isEmpty()) {
            ColorBlobLocatorProcessor.Blob closestBlob = blobs.get(0);
            RotatedRect boxFit = closestBlob.getBoxFit();

            int blockX = (int) boxFit.center.x;
            int blockY = (int) boxFit.center.y;

            telemetry.addData("Closest Block Position:", "(%d, %d)", blockX, blockY);

            if (blockX < SCREEN_CENTER_X) {
                moveIntakeSlidesSpecific(pixelToExtensionScale * (SCREEN_CENTER_X - blockX);
            } else if (blockX > SCREEN_CENTER_X) {
                moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.MIN_RETRACTED);
            }

            // Adjust robot orientation based on Y position
            if (blockY < SCREEN_CENTER_Y) {
                moveIntakeSlidesForward();
            } else {
                moveIntakeSlidesBackward();
            }
        } else {
            telemetry.addLine("No block detected of specified color.");
        }
        telemetry.update();
    }
    */

    public void moveIntakeSlidesSpecific(double extension){
        intakeSlideServoLeft.setPosition(extension);
        intakeSlideServoRight.setPosition(extension);
        intakeSlidesState.leftSlidePos = intakeSlideServoLeft.getPosition();
        intakeSlidesState.rightSlidePos = intakeSlideServoRight.getPosition();
    }

    public void moveIntakeSlidesForward(){
        double intakeSlideServoPosition = intakeSlideServoLeft.getPosition();

        if (intakeSlideServoPosition < INTAKE_SLIDES_STATE.MAX_EXTENSION.leftSlidePos) {
            intakeSlideServoLeft.setPosition(intakeSlideServoPosition + INTAKE_SLIDE_DELTA);
            intakeSlideServoRight.setPosition(intakeSlideServoPosition + INTAKE_SLIDE_DELTA);
        }
    }

    public void moveIntakeSlidesBackward() {
        double intakeSlideServoPosition = intakeSlideServoLeft.getPosition();

        if (intakeSlideServoPosition > INTAKE_SLIDES_STATE.MIN_RETRACTED.leftSlidePos) {
            intakeSlideServoLeft.setPosition(intakeSlideServoPosition - INTAKE_SLIDE_DELTA);
            intakeSlideServoRight.setPosition(intakeSlideServoPosition - INTAKE_SLIDE_DELTA);
        }
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
