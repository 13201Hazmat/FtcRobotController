package org.firstinspires.ftc.teamcode.Controllers;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

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
import org.firstinspires.ftc.vision.opencv.ColorRange;


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
    public double calculateIntakeSlideExtension(ColorRange targetColor){
        vision.locateNearestSamplefromRobot(targetColor);
        return vision.xExtensionFactor;
    }

    public void extendIntakeArmToPrePickupByColor(ColorRange targetColor){
        if (vision.targetBlobDetected) {
            intakeSlides.moveIntakeSlidesSpecific(
                    calculateIntakeSlideExtension(targetColor));
        } else {
            intakeSlides.moveIntakeSlidesToRange(1.0);
        }
        intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        safeWaitMilliSeconds(200 + 100* intakeSlides.slideExtensionFactor());
    }

    public void extendIntakeArmAndPickUpSampleByColor(ColorRange targetColor) {
        extendIntakeArmToPrePickupByColor(targetColor);
        pickupSequence();
        //TODO: ADD CODE FOR COLOR SENSOR REJECT
    }

    public Action extendIntakeArmAndPickUpSampleByColorAction(ColorRange targetColor) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                extendIntakeArmAndPickUpSampleByColor(targetColor);
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

    public void extendIntakeArmToPrePickupByExtensionFactor(double extensionFactor){
        intakeSlides.moveIntakeSlidesToRange(extensionFactor);
        intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        safeWaitMilliSeconds(200 + 100* intakeSlides.slideExtensionFactor());
    }



    public Action extendIntakeArmToPrePickupByExtensionFactorAction(double extensionFactor) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                extendIntakeArmToPrePickupByExtensionFactor(extensionFactor);
                return false;
            }
        };
    }


    public void extendIntakeArmSwivelToPrePickupByExtensionFactor(double extensionFactor, double swivelDegrees){
        intakeSlides.moveIntakeSlidesToRange(extensionFactor);
        intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        intakeArm.moveSwivelTo(swivelDegrees);
        safeWaitMilliSeconds(200 + 100* intakeSlides.slideExtensionFactor());
    }

    public Action extendIntakeArmSwivelToPrePickupByExtensionFactorAction(double extensionFactor, double swivelDegrees) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeSlides.moveIntakeSlidesToRange(extensionFactor);
                intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                intakeArm.moveSwivelTo(swivelDegrees);
                return false;
            }
        };
    }

    public Action moveIntakeArmToAction(IntakeArm.ARM_STATE toArmState) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakeArm.moveArm(toArmState);
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
                intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
                return false;
            }
        };
    }

    public void pickupSequence(){
        intakeArm.moveArm(IntakeArm.ARM_STATE.PICKUP);
        safeWaitMilliSeconds(200);
        intakeArm.closeGrip();
        safeWaitMilliSeconds(100);
        intakeArm.moveArm(IntakeArm.ARM_STATE.PRE_PICKUP);
        safeWaitMilliSeconds(100);

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

    public Action pickupSequenceAction1() {
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

    public Action moveOuttakeToTransferAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                moveOuttakeToTransfer();
                return false;
            }
        };

    }

    public void transferSampleFromIntakePreTransferToOuttakePreDrop(){
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        safeWaitMilliSeconds(200+ 150* intakeSlides.slideExtensionFactor());//100
        intakeArm.moveArm(IntakeArm.ARM_STATE.TRANSFER);
        safeWaitMilliSeconds(800);//800
        intakeArm.openGrip();
        safeWaitMilliSeconds(400);//500
        intakeArm.moveArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        safeWaitMilliSeconds(200);
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
                safeWaitMilliSeconds(200);
                return false;
            }
        };

    }

    public Action transferSampleFromIntakePreTransferToOuttakePreDropAction1() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                moveIntakeSlidesToAction(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED),
                                new SleepAction(0.200+ 0.100* intakeSlides.slideExtensionFactor()),
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.TRANSFER),
                                new SleepAction(0.8),
                                openIntakeGripAction(),
                                new SleepAction(0.4),
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.POST_TRANSFER),
                                new SleepAction(0.2),
                                moveOuttakeArmToAction(Outtake.ARM_STATE.DROP),
                                new SleepAction(0.2)
                        )
                );
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
                outtake.moveArm(toOuttakeArmState);
                return false;
            }
        };
    }

    //Drop Actions
    public void moveOuttakeSlidesTo(Outtake.SLIDE_STATE toOuttakeState) {
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
                moveOuttakeSlidesTo(toOuttakeState);
                return false;
            }
        };
    }

    public void dropSamplefromOuttake() {
        outtake.moveWristDrop();
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
                return false;
            }
        };
    }

    public void dropSamplefromOuttakeOnly() {
        outtake.moveWristDrop();
        safeWaitMilliSeconds(500);
    }

    public Action moveOuttakeWristDrop() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                outtake.moveWristDrop();
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
                                moveOuttakeWristDrop(),
                                new SleepAction(0.5)
                        )
                );
                return false;
            }
        };
    }

    public Action pickSampleActionByColor(ColorRange targetColor) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                extendIntakeArmAndPickUpSampleByColor(targetColor);
                transferSampleFromIntakePreTransferToOuttakePreDrop();
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

    public Action pickSampleToOuttakePreDropAction1() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                Actions.runBlocking(
                        new SequentialAction(
                                pickupSequenceAction1(),
                                closeIntakeGripAction(),
                                moveIntakeArmToAction(IntakeArm.ARM_STATE.TRANSFER),
                                transferSampleFromIntakePreTransferToOuttakePreDropAction1()
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
                safeWaitTillOuttakeSlideStateMilliSeconds(800, Outtake.SLIDE_STATE.HIGH_BUCKET);
                safeWaitMilliSeconds(200);
                dropSamplefromOuttake();
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
                outtake.moveArm(Outtake.ARM_STATE.DROP);
                moveOuttakeSlidesTo(Outtake.SLIDE_STATE.HIGH_BUCKET);
                //safeWaitTillOuttakeSlideStateMilliSeconds(800, Outtake.SLIDE_STATE.HIGH_BUCKET);
                //safeWaitMilliSeconds(200);
                //dropSamplefromOuttake();
                return false;
            }
        };
    }

    public Action safeWaitTillOuttakeSlideStateMilliSecondsAction() {
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
                if (outtake.isOuttakeSlidesInState(Outtake.SLIDE_STATE.HIGH_BUCKET)) {
                    return false;
                } else {
                    safeWaitMilliSeconds(25);
                    return true;
                }
            }
        };
    }


    public void setToAutoEndStateObservationZonePark() {
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.MAX_EXTENSION);
        intakeArm.moveArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);
        outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
    }

    public Action setToAutoEndStateObservationZoneParkAction() {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                setToAutoEndStateObservationZonePark();
                return false;
            }
        };
    }

    public void setToAutoEndStateSubmerssiblePark() {
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
        intakeArm.moveArm(IntakeArm.ARM_STATE.POST_TRANSFER);
        outtake.moveArm(Outtake.ARM_STATE.DROP);
        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
    }

    public void setToAutoEndStateSpecimenPark(){
        intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.MAX_EXTENSION);
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


    public void safeWaitTillOuttakeSlideStateMilliSeconds(double timeoutTime, Outtake.SLIDE_STATE toSlideState) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < timeoutTime &&
                !(outtake.isOuttakeSlidesInState(toSlideState))) {
        }
    }
    
    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }

    /*public Action safeWaitMilliSecondsAction(double time) {
        return new Action() {
            @Override
            public void preview(Canvas canvas) {
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime(MILLISECONDS);
                timer.reset();
                if (!currentOpMode.isStopRequested() && timer.time() < time) {
                    safeWaitMilliSeconds(25);
                    return true;
                } else {
                    return false;
                }
            }
        };
    }*/

}