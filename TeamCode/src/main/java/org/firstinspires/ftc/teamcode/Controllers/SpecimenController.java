package org.firstinspires.ftc.teamcode.Controllers;



import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

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

    public void closeGripAndMoveTo(SpecimenHandler.SPECIMEN_SLIDE_STATE toSlideState){
        if (specimenHandler.gripState == SpecimenHandler.SPECIMEN_GRIP_STATE.OPEN) {
            specimenHandler.closeGrip();
            safeWaitMilliSeconds(200);
        }
        specimenHandler.moveSpecimenSlides(toSlideState);
    }

    public Action closeGripAndMoveToAction(SpecimenHandler.SPECIMEN_SLIDE_STATE toSlideState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                closeGripAndMoveTo(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER);
                return false;
            }
        };
    }

    public void latchAndOpenGripAndMoveToPickup(){
        specimenHandler.lowerSlideToLatch();
        if (specimenHandler.autoOpenSpecimenGrip) {
            safeWaitMilliSeconds(300);
            specimenHandler.openGrip();
            safeWaitMilliSeconds(200);
            specimenHandler.moveSpecimenSlides(SpecimenHandler.SPECIMEN_SLIDE_STATE.PICKUP);
        }
    }

    public Action latchAndOpenGripAndMoveToPickupAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                latchAndOpenGripAndMoveToPickup();
                return false;
            }
        };
    }

    public void moveToPickup(){
        specimenHandler.moveSpecimenSlides(SpecimenHandler.SPECIMEN_SLIDE_STATE.PICKUP);

    }

    public Action moveToPickupAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveToPickup();
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

}
