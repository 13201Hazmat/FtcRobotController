package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


import org.firstinspires.ftc.teamcode.SubSystems.Outtake;



public class OuttakeController {

    //Create object reference to objects to systems passed from TeleOp
    public Outtake outtake;

    public OuttakeController(Outtake outtake, LinearOpMode currentOpMode) {
        this.outtake = outtake;
    }

    public void placeSampleInHighBucket(){
        outtake.moveOuttakeSlides(Outtake.OUTTAKE_SLIDE_STATE.HIGH_BUCKET);
        outtake.moveArm(Outtake.OUTTAKE_ARM_STATE.DROP);
        safeWaitMilliSeconds(500);
        outtake.moveWristDrop();
    }

    public Action placeSampleInHighBucketAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                placeSampleInHighBucket();
                return false;
            }
        };
    };

    public void lowerToTransfer() {
        outtake.moveOuttakeSlides(Outtake.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtake.moveArm(Outtake.OUTTAKE_ARM_STATE.TRANSFER);
    }

    public Action lowerToTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                lowerToTransfer();
                return false;
            }
        };
    };

    public void dropSampleToObservationZone() {
        outtake.moveArm(Outtake.OUTTAKE_ARM_STATE.DROP);
        outtake.moveWristDrop();
        safeWaitMilliSeconds(1000);
        outtake.moveArm(Outtake.OUTTAKE_ARM_STATE.TRANSFER);
    }

    public Action dropSampleToObservationZoneAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                dropSampleToObservationZone();
                return false;
            }
        };
    };

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }
    volatile boolean stopRequested = false;
}