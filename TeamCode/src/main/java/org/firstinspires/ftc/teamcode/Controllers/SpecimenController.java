package org.firstinspires.ftc.teamcode.Controllers;



import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.SpecimenHandler;



public class SpecimenController {
    public SpecimenHandler specimenHandler;

    LinearOpMode currentOpMode;

    public SpecimenController(SpecimenHandler specimenHandler, LinearOpMode currentOpMode) {
        this.specimenHandler = specimenHandler;

        this.currentOpMode = currentOpMode;
    }

    public void hangSpecimenAtStart() {
        specimenHandler.moveSpecimenSlides(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER);
        specimenHandler.lowerSlideToLatch();
        specimenHandler.openGrip();
    }

    public Action hangSpecimenAtStartAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                hangSpecimenAtStartAction();
                return false;
            }
        };
    }

    public void lowerSpecimenbackInitAfterHang() {
        specimenHandler.backToInit();
    }


    public Action lowerSpecimenbackInitAfterHangAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                lowerSpecimenbackInitAfterHangAction();
                return false;
            }
        };
    }
    public void pickupSpecimenFromObservation() {
        //TODO add code to turn 180 and align specimenHandler with specimen
        safeWaitSeconds(0.500);
        specimenHandler.closeGrip();
        specimenHandler.moveSpecimenSlides(SpecimenHandler.SPECIMEN_SLIDE_STATE.HIGH_CHAMBER);
    }



    public Action pickupSpecimenFromObservationAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                pickupSpecimenFromObservation();
                return false;
            }
        };
    }
    public void hangSpecimenTopChamber() {
        specimenHandler.lowerSlideToLatch();
        safeWaitSeconds(.500);
        specimenHandler.openGrip();
        specimenHandler.backToInit();
    }



    public Action hangSpecimenTopChamberAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                hangSpecimenTopChamber();
                return false;
            }
        };
    }


    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }
    volatile boolean stopRequested = false;


}
