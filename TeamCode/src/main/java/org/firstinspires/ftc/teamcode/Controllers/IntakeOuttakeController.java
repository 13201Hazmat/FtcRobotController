package org.firstinspires.ftc.teamcode.Controllers;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.opencv.ColorRange;


public class IntakeOuttakeController {
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public Vision vision;
    LinearOpMode currentOpMode;


    public IntakeOuttakeController(IntakeArm intakeArm, IntakeSlides intakeSlides, Outtake outtake, Vision vision, LinearOpMode currentOpMode) {
        this.intakeArm = intakeArm;
        this.intakeSlides = intakeSlides;
        this.outtake = outtake;
        this.vision = vision;
        this.currentOpMode = currentOpMode;
    }

    //Intake Actions
    public double calculateIntakeSlideExtension(ColorRange targetColor){
        vision.locateNearestSamplefromRobot(targetColor);
        return vision.xExtensionFactor;
    }

    public void extendIntakeArmToPrePickup(ColorRange targetColor){
        if (vision.targetBlobDetected) {
            intakeSlides.moveIntakeSlidesSpecific(
                    calculateIntakeSlideExtension(targetColor));
        } else {
            intakeSlides.moveIntakeSlidesToRange(1.0);
        }
        intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        safeWaitMilliSeconds(200 + 100* intakeSlides.slideExtensionFactor());
    }

    public void pickupSequence(){
        intakeArm.moveArm(IntakeArm.ARM_STATE.PICKUP);
        safeWaitMilliSeconds(200);
        intakeArm.closeGrip();
        safeWaitMilliSeconds(100);
        intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        safeWaitMilliSeconds(100);
    }

    public void extendArmAndPickUpSample(ColorRange targetColor) {
        extendIntakeArmToPrePickup(targetColor);
        pickupSequence();
        //TODO: ADD CODE FOR COLOR SENSOR REJECT
    }

    public Action extendArmAndPickUpSampleAction(ColorRange targetColor) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                extendArmAndPickUpSample(targetColor);
                return false;
            }
        };

    }

    //Transfer Actions
    public void closeGripAndMoveIntakeArmToPreTransfer(){
        intakeArm.closeGrip();
        intakeArm.moveArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);
    }

    public void moveOuttakeToTransfer(){
        if (!outtake.isOuttakeInTransfer()) {
            outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
            outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
            safeWaitMilliSeconds(500);
        }
    }

    public void transferSampleFromIntakePreTransferToOuttakePreDrop(){
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        safeWaitMilliSeconds(100* intakeSlides.slideExtensionFactor());
        intakeArm.moveArm(IntakeArm.ARM_STATE.TRANSFER);
        safeWaitMilliSeconds(600);
        intakeArm.openGrip();
        safeWaitMilliSeconds(500);
        intakeArm.moveArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        safeWaitMilliSeconds(300);
        outtake.moveArm(Outtake.ARM_STATE.DROP);
        safeWaitMilliSeconds(200);
    }

    public Action transferSampleFromIntakePreTransferToOuttakePreDropAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                transferSampleFromIntakePreTransferToOuttakePreDrop();
                return false;
            }
        };

    }

    //Drop Actions
    public void moveOuttakeTo(Outtake.SLIDE_STATE toOuttakeState) {
        outtake.moveOuttakeSlides(toOuttakeState);
        safeWaitMilliSeconds(300);
    }

    public Action moveOuttakeToAction(Outtake.SLIDE_STATE toOuttakeState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeTo(toOuttakeState);
                return false;
            }
        };
    }

    public void dropSamplefromOuttake() {
        outtake.moveWristDrop();
        safeWaitMilliSeconds(750);
        outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
        safeWaitMilliSeconds(200);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
    }

    public Action dropSamplefromOuttakeAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                dropSamplefromOuttake();
                return false;
            }
        };
    }

    public Action pickSampleAction(ColorRange targetColor) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                extendArmAndPickUpSampleAction(targetColor);
                transferSampleFromIntakePreTransferToOuttakePreDropAction();
                return false;
            }
        };
    }

    public Action dropHighBucketAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeToAction(Outtake.SLIDE_STATE.HIGH_BUCKET);
                dropSamplefromOuttakeAction();
                return false;
            }
        };
    }


    
    public void setToAutoEndState() {
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        intakeArm.moveArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
    }

    public Action setToAutoEndStateAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                setToAutoEndState();
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