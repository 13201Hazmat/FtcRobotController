package org.firstinspires.ftc.teamcode.Controllers;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;


public class IntakeOuttakeController {
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Outtake outtake;
    public Vision vision;
    LinearOpMode currentOpMode;

    public boolean autoTransferEnabled = false;
    public boolean initiateAutoTransfer = false;

    public IntakeOuttakeController(IntakeArm intakeArm, IntakeSlides intakeSlides, Outtake outtake, Vision vision, LinearOpMode currentOpMode) {
        this.intakeArm = intakeArm;
        this.intakeSlides = intakeSlides;
        this.outtake = outtake;
        this.vision = vision;
        this.currentOpMode = currentOpMode;
    }

    //Intake Actions

    public void moveIntakeArm(IntakeArm.ARM_STATE toIntakeArmState){
        intakeArm.moveWristAndSwivel(toIntakeArmState);
        if (toIntakeArmState == IntakeArm.ARM_STATE.TRANSFER) {
            safeWaitMilliSeconds(200);
        }
        intakeArm.moveArm(toIntakeArmState);
    }


    public Action moveIntakeArmToAction(IntakeArm.ARM_STATE toArmState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveIntakeArm(toArmState);
                return false;
            }
        };

    }
    public Action moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE toSlideState){
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeSlides.moveIntakeSlides(toSlideState);
                return false;
            }
        };
    }

    public Action extendIntakeArmByExtensionFactorAction(double extensionFactor) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeSlides.moveIntakeSlidesToRange(extensionFactor);
                return false;
            }
        };
    }

    public Action extendIntakeArmSwivelToPrePickupByExtensionFactorAction(double extensionFactor, double swivelDegrees) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeSlides.moveIntakeSlidesToRange(extensionFactor);
                moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                intakeArm.moveSwivelTo(swivelDegrees);
                return false;
            }
        };
    }

    public Action openIntakeGripAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeArm.openGrip();
                return false;
            }
        };
    }

    public Action closeIntakeGripAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeArm.closeGrip();
                return false;
            }
        };
    }

    //Outtake
    public Action openOuttakeGripAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.openGrip();
                return false;
            }
        };
    }

    public Action closeOuttakeGripAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.closeGrip();
                return false;
            }
        };
    }

    //Drop Actions
    public void moveOuttakeSlidesTo(Outtake.SLIDE_STATE toOuttakeState) {
        outtake.moveOuttakeSlides(toOuttakeState);
        safeWaitTillOuttakeSlideStateMilliSecondsAction(toOuttakeState);
        //safeWaitMilliSeconds(300);
    }

    public Action moveOuttakeSlidesToAction(Outtake.SLIDE_STATE toOuttakeState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeSlidesTo(toOuttakeState);
                return false;
            }
        };
    }



    public void moveOuttakeArm(Outtake.ARM_STATE toOuttakeArmState){
        outtake.moveArm(toOuttakeArmState);
    }

    public Action moveOuttakeArmToAction(Outtake.ARM_STATE toOuttakeArmState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.moveArm(toOuttakeArmState);
                return false;
            }
        };
    }


    public Action moveOuttakeToAction(Outtake.ARM_STATE toOuttakeArmState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new ParallelAction(
                                moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.TRANSFER),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.TRANSFER),
                                openOuttakeGripAction()
                        )
                );
                return false;
            }
        };

    }


    public void moveOuttakeToTransfer(){
        if (!outtake.isOuttakeInTransfer()) {
            outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
            outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
            outtake.openGrip();
            safeWaitMilliSeconds(500);
        }
    }

    public Action moveOuttakeToTransferAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new ParallelAction(
                                moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.TRANSFER),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.TRANSFER),
                                openOuttakeGripAction()
                        )
                );
                return false;
            }
        };

    }

    public void moveOuttakeToHighChamber(){
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_CHAMBER);
        outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
        safeWaitMilliSeconds(1000);
    }

    public Action moveOuttakeToHighChamberAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.HIGH_CHAMBER),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.HIGH_CHAMBER)//,
                                //new SleepAction(1)
                        )
                );
                return false;
            }
        };
    }

    public Action moveOuttakeToHighChamberDropSpecimenAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                moveOuttakeToHighChamberAction(),
                                new SleepAction(0.2),
                                dropSamplefromOuttakeAction(),
                                new SleepAction(0.2),
                                moveOuttakeToTransferAction()
                        )
                );
                return false;
            }
        };
    }

    public Action moveOuttakeHighBucketAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.moveArm(Outtake.ARM_STATE.DROP);
                moveOuttakeSlidesTo(Outtake.SLIDE_STATE.HIGH_BUCKET);
                return false;
            }
        };
    }

    public Action resetOuttakeSlidesAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.resetOuttakeMotorMode();
                return false;
            }
        };
    }

    public void pickupSequence(){
        moveIntakeArm(IntakeArm.ARM_STATE.PICKUP);
        safeWaitMilliSeconds(200);
        intakeArm.closeGrip();
        safeWaitMilliSeconds(100);
        moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        safeWaitMilliSeconds(100);

    }

    public Action pickupSequenceAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.PICKUP),
                                new SleepAction(0.2),
                                closeIntakeGripAction(),
                                new SleepAction(0.1),
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.PRE_PICKUP)
                        )
                );
                return false;
            }
        };
    }


    public void transferSampleFromIntakePreTransferToOuttakeTransfer(){
        moveOuttakeArm(Outtake.ARM_STATE.PRE_TRANFER);//Added
        moveIntakeArm(IntakeArm.ARM_STATE.TRANSFER);
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        safeWaitMilliSeconds(200+ 200* intakeSlides.slideExtensionFactor());//100
        //moveArm(IntakeArm.ARM_STATE.TRANSFER);
        outtake.senseOuttakeSampleColor();
        //safeWaitMilliSeconds(700);//500
        safeWaitTillOuttakeSensorSensedMilliSeconds(400);
        moveOuttakeArm(Outtake.ARM_STATE.TRANSFER);//Added
        safeWaitMilliSeconds(100);
        outtake.closeGrip();
        safeWaitMilliSeconds(200);
        intakeArm.openGrip();
        safeWaitMilliSeconds(100);//400
        moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        safeWaitMilliSeconds(200);
    }

    public Action transferSampleFromIntakePreTransferToOuttakeTransferAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                moveOuttakeArmToAction(Outtake.ARM_STATE.PRE_TRANFER),
                                moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED),
                                new SleepAction(0.200+ 0.100* intakeSlides.slideExtensionFactor()),
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.TRANSFER),
                                new SleepAction(0.2),
                                moveOuttakeToAction(Outtake.ARM_STATE.TRANSFER),
                                new SleepAction(0.1),
                                closeOuttakeGripAction(),//TODO : Fix same as teleop
                                new SleepAction(0.8),
                                openIntakeGripAction(),
                                new SleepAction(0.4),
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.POST_TRANSFER),
                                new SleepAction(0.2)
                        )
                );
                return false;
            }
        };
    }

    public void transferSampleFromIntakePreTransferToOuttakePreDrop(){
        transferSampleFromIntakePreTransferToOuttakeTransfer();
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
                Actions.runBlocking(
                        new SequentialAction(
                                transferSampleFromIntakePreTransferToOuttakeTransferAction(),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.DROP),
                                new SleepAction(0.2)
                        )
                );
                return false;
            }
        };
    }

    public void dropSamplefromOuttake() {
        outtake.openGrip();
        safeWaitMilliSeconds(500);
        outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
        safeWaitTillOuttakeSlideStateMilliSeconds(500, Outtake.SLIDE_STATE.TRANSFER);
    }

    public Action dropSamplefromOuttakeAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                dropSamplefromOuttake();
                Actions.runBlocking(
                        new SequentialAction(
                                openOuttakeGripAction(),
                                new SleepAction(500),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.TRANSFER),
                                moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.TRANSFER),
                                safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.TRANSFER)
                        )
                );
                return false;
            }
        };
    }

    public Action dropSamplefromOuttakeOnlyAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                openOuttakeGripAction(),
                                new SleepAction(0.5)
                        )
                );
                return false;
            }
        };
    }



    public void setToAutoEndStateSubmerssiblePark() {
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        outtake.moveArm(Outtake.ARM_STATE.DROP);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
    }

    public Action setToAutoEndStateSubmerssibleParkAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                setToAutoEndStateSubmerssiblePark();
                return false;
            }
        };
    }

    public void safeWaitTillOuttakeSensorSensedMilliSeconds(double timeoutTime) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        outtake.senseOuttakeSampleColor();
        while (!currentOpMode.isStopRequested() && timer.time() < timeoutTime &&
                !(outtake.outtakeSampleSensed)) {
            outtake.senseOuttakeSampleColor();
        }
    }

    public void safeWaitTillOuttakeSlideStateMilliSeconds(double timeoutTime, Outtake.SLIDE_STATE toSlideState) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < timeoutTime &&
                !(outtake.isOuttakeSlidesInState(toSlideState))) {
        }
    }

    public Action safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE toOuttakeSlideState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                /*ElapsedTime timer = new ElapsedTime(MILLISECONDS);
                timer.reset();
                while (!currentOpMode.isStopRequested() && timer.time() < 800 &&
                        !(outtake.isOuttakeSlidesInState(Outtake.SLIDE_STATE.HIGH_BUCKET))) {
                    safeWaitMilliSeconds(25);
                }
                return false;
                 */
                if (outtake.isOuttakeSlidesInState(toOuttakeSlideState)) {
                    return false;
                } else {
                    safeWaitMilliSeconds(25);
                    return true;
                }
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