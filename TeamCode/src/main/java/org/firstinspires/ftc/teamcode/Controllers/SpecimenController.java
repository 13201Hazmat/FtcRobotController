package org.firstinspires.ftc.teamcode.Controllers;



import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.content.pm.LabeledIntent;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SubSystems.SpecimenHandler;



public class SpecimenController {
    public SpecimenHandler specimenHandler;

    LinearOpMode currentOpMode;

    public SpecimenController(SpecimenHandler specimenHandler, LinearOpMode currentOpMode) {
        this.specimenHandler = specimenHandler;
        this.currentOpMode = currentOpMode;
    }

    public void closeGripAndMoveTo(SpecimenHandler.SLIDE_STATE toSlideState){
        if (specimenHandler.gripState == SpecimenHandler.GRIP_STATE.OPEN) {
            specimenHandler.closeGrip();
            safeWaitMilliSeconds(200);
        }
        specimenHandler.moveSpecimenSlides(toSlideState);
    }

    public Action closeGripAndMoveToAction(SpecimenHandler.SLIDE_STATE toSlideState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                closeGripAndMoveTo(toSlideState);
                return false;
            }
        };
    }

    public void latchAndOpenGripAndMoveTo(SpecimenHandler.SLIDE_STATE toSlideState){
        specimenHandler.lowerSlideToLatch();
        if (specimenHandler.autoOpenSpecimenGrip) {
            safeWaitTillLatchMilliSeconds(500);
            specimenHandler.openGrip();
            //safeWaitMilliSeconds(200);
            specimenHandler.moveSpecimenSlides(toSlideState);
        }
    }

    public Action latchAndOpenGripAndMoveToAction(SpecimenHandler.SLIDE_STATE toSlideState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                latchAndOpenGripAndMoveTo(toSlideState);
                return false;
            }
        };
    }

    public void moveTo(SpecimenHandler.SLIDE_STATE toSlideState){
        specimenHandler.moveSpecimenSlides(toSlideState);
    }

    public Action moveToAction(SpecimenHandler.SLIDE_STATE toSlideState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveTo(toSlideState);
                return false;
            }
        };
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }

    public void safeWaitTillLatchMilliSeconds(double timeoutTime) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < timeoutTime &&
                !(specimenHandler.isOuttakeSlidesInState(SpecimenHandler.SLIDE_STATE.HICH_CHAMBER_LATCH) ||
                        specimenHandler.isOuttakeSlidesInState(SpecimenHandler.SLIDE_STATE.MIN_RETRACTED_LOW_CHAMBER_LATCH))) {
        }
    }

}
