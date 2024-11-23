package org.firstinspires.ftc.teamcode.Controllers;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;



public class IntakeController {
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    LinearOpMode currentOpMode;


    public IntakeController(IntakeArm intakeArm, IntakeSlides intakeSlides, LinearOpMode currentOpMode) {
        this.intakeArm = intakeArm;
        this.intakeSlides = intakeSlides;
        this.currentOpMode = currentOpMode;
    }


    public void IntakeSampleAtStart() {
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP);
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.IN_BETWEEN);
        safeWaitSeconds(1);
        //TODO add code to actually pick up block
        safeWaitSeconds(1);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        safeWaitSeconds(1);
        //TODO add code for transfer
    }

    public Action IntakeSampleAtStartAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                IntakeSampleAtStart();
                return false;
            }
        };

    }
    public void IntakeSampleAtYS1() {
        intakeSlides.moveIntakeSlidesSpecific(0.2);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP);

        //TODO add code to actually pick up block
        safeWaitSeconds(1);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);

        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        safeWaitSeconds(1);
        //TODO add code for transfer



    }

    public Action IntakeSampleAtYS1Action() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                IntakeSampleAtYS1();
                return false;
            }
        };

    }
    public void IntakeSampleAtYS2() {
        intakeSlides.moveIntakeSlidesSpecific(0.2);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP);

        //TODO add code to actually pick up block
        safeWaitSeconds(1);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);

        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        safeWaitSeconds(1);
        //TODO add code for transfer



    }

    public Action IntakeSampleAtYS2Action() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                IntakeSampleAtYS2();
                return false;
            }
        };

    }
    public void IntakeSampleAtColor1() {
        intakeSlides.moveIntakeSlidesSpecific(0.2);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP);

        //TODO add code to actually pick up block
        safeWaitSeconds(1);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);

        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        safeWaitSeconds(1);
        //TODO add code for transfer



    }

    public Action IntakeSampleAtColor1Action() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                IntakeSampleAtYS2();
                return false;
            }
        };

    }
    public void IntakeSampleAtColor2() {
        intakeSlides.moveIntakeSlidesSpecific(0.2);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP);

        //TODO add code to actually pick up block
        safeWaitSeconds(1);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);

        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        safeWaitSeconds(1);
        //TODO add code for transfer



    }

    public Action IntakeSampleAtColor2Action() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                IntakeSampleAtYS2();
                return false;
            }
        };

    }
    //for safe wait
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