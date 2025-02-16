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


import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
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

    public void moveIntakeArmAndSlidesToTransfer() {
        if (intakeSlides.intakeSlidesState == IntakeSlides.SLIDES_STATE.MAX_EXTENSION) {
            intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.MAX_EXTENSION);
            safeWaitMilliSeconds(50);
        }
        //moveIntakeArmAndSlidesToTransfer();
        intakeArm.moveArm(IntakeArm.ARM_STATE.TRANSFER);
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        safeWaitMilliSeconds(100+ 150* intakeSlides.slideExtensionFactor());
    }

    public Action moveIntakeArmAndSlidesToTransferAction(){
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveIntakeArmAndSlidesToTransfer();
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
                moveIntakeArmAndSlidesToTransfer();
                //intakeSlides.moveIntakeSlides(toSlideState);
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
                moveIntakeArm(IntakeArm.ARM_STATE.LOWER_PRE_PICKUP); //LOWER_PRE_PICKUP
                intakeArm.moveSwivelTo(swivelDegrees);
                return false;
            }
        };
    }

    public Action extendIntakeArmByVisionAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                vision.locateNearestSampleFromRobot();
                intakeSlides.moveIntakeSlidesToRange(vision.yExtensionFactor);
                moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                if (vision.angle < 45 ) {
                    intakeArm.moveSwivelCentered();
                } else {
                    intakeArm.moveSwivelPerpendicular();
                }
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
        if (outtake.outtakeArmState == Outtake.ARM_STATE.SPECIMEN_PICKUP
                && toOuttakeArmState == Outtake.ARM_STATE.PRE_TRANSFER) {
            outtake.moveWrist(Outtake.ARM_STATE.INIT);
            safeWaitMilliSeconds(100);
        }
        outtake.moveArm(toOuttakeArmState);
    }

    public Action moveOuttakeArmOnlyToAction(Outtake.ARM_STATE toOuttakeArmState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeArm(toOuttakeArmState);
                return false;
            }
        };
    }

    public Action moveOuttakeArmToAction(Outtake.ARM_STATE toOuttakeArmState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                moveOuttakeArmOnlyToAction(toOuttakeArmState),
                                new SleepAction(0.3)
                        )
                );
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


    public void moveOuttakeToPreTransfer(){
        if (outtake.outtakeArmState == Outtake.ARM_STATE.SPECIMEN_PICKUP) {
            outtake.moveWrist(Outtake.ARM_STATE.INIT);
            safeWaitMilliSeconds(100);
            moveOuttakeArm(Outtake.ARM_STATE.PRE_TRANSFER);
            safeWaitMilliSeconds(400);
        }
        if (!outtake.isOuttakeInPreTransfer()) {
            //outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER); // TODO: TRY TRANSFERING FREELY
            outtake.moveOuttakeSlidesToTransfer();
            outtake.moveArm(Outtake.ARM_STATE.PRE_TRANSFER);
            outtake.moveWrist(Outtake.ARM_STATE.PRE_TRANSFER);
            outtake.openGrip();
            safeWaitMilliSeconds(500);
        }
    }

    public Action moveOuttakeToPreTransferAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }
            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                    moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.TRANSFER),
                                    safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.TRANSFER)
                                ),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.PRE_TRANSFER),
                                openOuttakeGripAction()
                        )
                );
                return false;
            }
        };

    }

    public void moveOuttakeToSpecimenPickUp() {
        if (intakeSlides.intakeSlidesState != IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED) {
            moveIntakeArmAndSlidesToTransfer();
        }
        if (intakeArm.intakeArmState != IntakeArm.ARM_STATE.SPECIMEN_PICKUP) {
            intakeArm.moveArm(IntakeArm.ARM_STATE.SPECIMEN_PICKUP);
            safeWaitMilliSeconds(200);
        }
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.SPECIMEN_PICKUP);
        outtake.moveArm(Outtake.ARM_STATE.SPECIMEN_PICKUP);
        outtake.adjustOpenGrip();
    }

    public Action moveOuttakeToSpecimenPickUpAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeToSpecimenPickUp();
                return false;
            }
        };
    }

    public void pickupSpecimenAndMoveOuttakeToHighChamber() {
        outtake.closeGrip();
        safeWaitMilliSeconds(100);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_CHAMBER);
        outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
    }

    public Action pickupSpecimenAndMoveOuttakeToHighChamberAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                pickupSpecimenAndMoveOuttakeToHighChamber();
                return false;
            }
        };
    }

    public void moveOuttakeToHighChamber(){
        if (outtake.outtakeSlidesState != Outtake.SLIDE_STATE.HIGH_CHAMBER ) {
            outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_CHAMBER);
            safeWaitMilliSeconds(300);
        }
        outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
        safeWaitMilliSeconds(300);
    }

    public Action moveOuttakeToHighChamberAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeToHighChamber();
                return false;
            }
        };
    }

    public void moveOuttakeToHighChamberLatch(){
        outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER_LATCH);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_CHAMBER_LATCH);
        safeWaitMilliSeconds(50);
    }

    public Action moveOuttakeToHighChamberLatchAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeToHighChamberLatch();
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
                                //new SleepAction(0.2),
                                dropSamplefromOuttakeAction(),
                                //new SleepAction(0.2),
                                moveOuttakeToPreTransferAction()
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
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        moveOuttakeSlidesToAction(Outtake.SLIDE_STATE.HIGH_BUCKET),
                                        safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE.HIGH_BUCKET)
                                ),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.AUTO_PRE_DROP)
                        )
                );
                return false;
            }
        };
    }

    public Action moveOuttakeHighBucketAction1() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.moveArm(Outtake.ARM_STATE.AUTO_PRE_DROP);
                moveOuttakeSlidesTo(Outtake.SLIDE_STATE.HIGH_BUCKET);
                safeWaitTillOuttakeSlideStateMilliSeconds(1500, Outtake.SLIDE_STATE.HIGH_BUCKET);
                //dropSamplefromOuttake();
                return false;
            }
        };
    }

    public Action safetyResetOuttakeSlidesAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.safetyReset();
                return false;
            }
        };
    }

    public Action resetOuttakeSlidesTouchAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (outtake.outtakeTouch.getState() == false) {
                    outtake.stopOuttakeMotors();
                    outtake.resetOuttakeMotorMode();
                }
                return false;
            }
        };
    }

    public void pickupLowerSequence(){
        moveIntakeArm(IntakeArm.ARM_STATE.PICKUP);
        safeWaitMilliSeconds(200);
        intakeArm.closeGrip();
        safeWaitMilliSeconds(100);
        moveIntakeArm(IntakeArm.ARM_STATE.LOWER_PRE_PICKUP);
        //moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
    }

    public Action pickupLowerSequenceAction() {
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
                                //moveIntakeArmToAction(IntakeArm.ARM_STATE.PRE_PICKUP)
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.LOWER_PRE_PICKUP),
                                new SleepAction(0.1)
                        )
                );
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
        //safeWaitMilliSeconds(100); TODO: PICKUP_SPEEDUP Check

    }

    public Action pickupSequenceAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                pickupSequence();
                return false;
            }
        };
    }

    public Action pickSampleToOuttakePreDropAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                pickupSequence();
                closeGripAndMoveIntakeArmToPreTransfer();
                transferSampleFromIntakePreTransferToOuttakePreDrop();
                return false;
            }
        };
    }

    public void closeGripAndMoveIntakeArmToPreTransfer(){
        intakeArm.closeGrip();
        intakeArm.moveArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);
    }

    public void transferSampleFromIntakePreTransferToOuttakeTransfer(){
        moveOuttakeArm(Outtake.ARM_STATE.PRE_TRANSFER);//Added
        moveIntakeArmAndSlidesToTransfer();
        /*moveIntakeArm(IntakeArm.ARM_STATE.TRANSFER);
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        safeWaitMilliSeconds(100+ 150* intakeSlides.slideExtensionFactor());//100*/
        safeWaitTillOuttakeSensorSensedMilliSeconds(400);
        moveOuttakeArm(Outtake.ARM_STATE.TRANSFER);//Added
        safeWaitMilliSeconds(100);
        outtake.closeGrip();
        safeWaitMilliSeconds(100);
        intakeArm.openGrip();
        safeWaitMilliSeconds(100);//400
        //moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        //safeWaitMilliSeconds(200);
    }

    public Action transferSampleFromIntakePreTransferToOuttakeTransferAction1() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                transferSampleFromIntakePreTransferToOuttakeTransfer();
                return false;
            }
        };
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
                                new ParallelAction(
                                    moveOuttakeArmToAction(Outtake.ARM_STATE.PRE_TRANSFER),
                                    moveIntakeArmAndSlidesToTransferAction()
                                    /*moveIntakeArmToAction(IntakeArm.ARM_STATE.TRANSFER),
                                    moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED)*/
                                ),
                                new SleepAction(0.200+ 0.100* intakeSlides.slideExtensionFactor()),
                                new SleepAction(0.2),
                                moveOuttakeToAction(Outtake.ARM_STATE.TRANSFER),
                                new SleepAction(0.1),
                                closeOuttakeGripAction(),//TODO : Fix same as teleop
                                new SleepAction(0.2),
                                openIntakeGripAction(),
                                new SleepAction(0.1),
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

        if (outtake.lastOuttakeSlideState == Outtake.SLIDE_STATE.HIGH_CHAMBER) {
            outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
        } else {
            outtake.moveArm(Outtake.ARM_STATE.DROP);
        };
        //safeWaitMilliSeconds(200);
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

    public void dropSamplefromOuttakeAndMoveArmToPreTransfer() {
        outtake.moveArm(Outtake.ARM_STATE.DROP);
        safeWaitMilliSeconds(200);
        outtake.openGrip();
        safeWaitMilliSeconds(200);
        outtake.moveArm(Outtake.ARM_STATE.PRE_TRANSFER);
        //outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER); // TODO: TRY TRANSFERING FREELY
        //outtake.moveOuttakeSlidesToTransfer();
        //safeWaitTillOuttakeSlideStateMilliSeconds(1500, Outtake.SLIDE_STATE.TRANSFER);
    }

    public Action dropSamplefromOuttakeAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                dropSamplefromOuttakeAndMoveArmToPreTransfer();
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

    public Action dropSamplefromOuttakeAndMoveArmToPreTransferAction1() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                dropSamplefromOuttakeAndMoveArmToPreTransfer();
                return false;
            }
        };
    }

    public Action moveOuttakeSlidesToTransferAction1() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.moveOuttakeSlidesToTransfer();
                safeWaitTillOuttakeSlideStateMilliSeconds(1500, Outtake.SLIDE_STATE.TRANSFER);
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
                outtake.openGrip();
                /*Actions.runBlocking(
                        new SequentialAction(
                                openOuttakeGripAction(),
                                new SleepAction(0.3)
                        )
                );*/
                return false;
            }
        };
    }



    public void setToAutoEndStateSubmerssiblePark() {
        moveIntakeArmAndSlidesToTransfer();
        //intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        //outtake.moveArm(Outtake.ARM_STATE.DROP);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
        outtake.moveArm(Outtake.ARM_STATE.AUTO_PARK);
        GameField.outtakeInParkPositionInAutonomous = true;
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
            intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        }
    }

    public void safeWaitTillOuttakeSlideStateMilliSeconds(double timeoutTime, Outtake.SLIDE_STATE toSlideState) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < timeoutTime &&
                !(outtake.isOuttakeSlidesInState(toSlideState))) {
        }
        outtake.safetyReset();
    }

    public Action safeWaitTillOuttakeSlideStateMilliSecondsAction(Outtake.SLIDE_STATE toOuttakeSlideState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (outtake.isOuttakeSlidesInState(toOuttakeSlideState)) {
                    outtake.safetyReset();
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