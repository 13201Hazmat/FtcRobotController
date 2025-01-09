package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;


/**
 * Defenition of the HzGamepad Class <BR>
 *
 * HzGamepad consists of system provided gamepad(s) and adds functionality to the selection
 * made on gamepads <BR>
 *
 * For Hazmat  PowerPlay, two Gamepads are used (gamepad1 and gamepad2) <BR>
 *
 * The controls are as follows: (replace with gamepad2 for 2nd gamepad) <BR>
 *  *      Left Stick for pan motion (gamepad1.left_stick_x and gamepad2.left_stick_y) <BR>
 *  *      Right Stick for turn motion (gamepad2.right_stick_x: gamepad1.right_stick_y) <BR>
 *  *      Right Bumper magazine flip and majorClaw state(gp2) (gamepad1.right_bumper, gamepad2.right_bumper) <BR>
 *  *      Left Bumper for spinner state and minorArm state(gp2) (gamepad1.left_bumper, gamepad2.left_bumper) <BR>
 *  *      Right Trigger for turbo, and majorArm Parking position(gp2) (gamepad1.right_trigger, gamepad2.right_trigger) <BR>
 *  *      Button A for elevator intake level and major arm pickup position(gp2) (gamepad1.a, gamepad2.a) <BR>
 *  *      Button Y for elevator level 2 and major arm capstone position(gp2) (gamepad1.y, gamepad2.y) <BR>
 *  *      Button X for elevator level 1 and majorArm down one level(gp2) (gamepad1.x, gamepad2.x) <BR>
 *  *      Button B for elevator level 3 and majorArm level up one(gp2) (gamepad1.b, gamepad2.b) <BR>
 *  *      Button Dpad_up for intake out & stop, also for minorArm level up one (gamepad1.dpad_up, gamepad2.dpad_up) <BR>
 *  *      Button Dpad_down for intake in & stop, also for minorArm level down one (gamepad1.dpad_down, gamepad2.dpad_down) <BR>
 *
 * To access the gamepad functions, use the gp1Get* or gp2Get* functions at the end of this class <BR>
 *     gp1GetLeftStickX(), gp2GetLeftStickX()
 *     gp1GetLeftStickY(), gp2GetLeftStickY()
 *     gp1GetRightStickX(), gp2GetRightStickX()
 *     gp1GetRightStickY(), gp2GetRightStickY()
 *     gp1GetLeftTrigger(), gp2GetRightTrigger()
 *     gp1GetLeftTriggerPress(), gp2GetRightTriggerPress for toggle value()
 *     gp1GetLeftBumper(), gp2GetRightBumper()
 *     gp1GetLeftBumperPress(), gp2GetRightBumperPress for toggle value()
 *     gp1GetX(), gp2GetY(), gp1GetA(), gp2GetB()
 *     gp1GetXPress(), gp2GetYPress(), gp1GetAPress(), gp2GetBPress() for toggle value()
 *     gp1GetDpad_up(), gp2GetDpad_down(). gp1GetDpad_left(), gp2GetDpad_right()
 *     gp1GetDpad_upPress(), gp2GetDpad_downPress(). gp1GetDpad_leftPress(), gp2GetDpad_rightPress()  for toggle value()
 *     gp1GetStart(), gp2GetStart()
 *
 */

public class GamepadController {

    //Create object reference to objects to systems passed from TeleOp
    public Gamepad hzGamepad1, hzGamepad2;
    //public Lights lights;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public IntakeOuttakeController intakeOuttakeController;
    public Outtake outtake;
    public Telemetry telemetry;
    LinearOpMode currentOpMode;

    /**
     * Constructor for HzGamepad1 and HzGamepad2 class that extends gamepad.
     * Assign the gamepad1 and gamepad2 given in OpMode to the gamepad used here.
     */
    public GamepadController(Gamepad hzGamepad1,
                             Gamepad hzGamepad2,
                             //Lights lights,
                             IntakeArm intakeArm,
                             IntakeSlides intakeSlides,
                             IntakeOuttakeController intakeOuttakeController,
                             Outtake outtake,
                             Telemetry telemetry,
                             LinearOpMode currentOpMode
    ) {
        this.hzGamepad1 = hzGamepad1;
        this.hzGamepad2 = hzGamepad2;
        this.intakeArm = intakeArm;
        this.intakeSlides = intakeSlides;
        this.intakeOuttakeController = intakeOuttakeController;
        this.outtake = outtake;
        //this.lights = lights;
        this.telemetry = telemetry;
        this.currentOpMode = currentOpMode;
    }

    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
        runIntake();
        runOuttake();
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }

    public boolean gp1DpadEnabled = false;
    private int circlePressCount = 0;
    private ElapsedTime circlePressTimer = new ElapsedTime(MILLISECONDS);
    private static final double CIRCLE_PRESS_TIMEOUT = 1000;


    public void runIntake() {
        if (gp1GetRightBumperPress()) {
            switch (intakeArm.intakeArmState) {
                case INIT:
                case EJECT_OR_PRE_TRANSFER:
                case LOWEST:
                case DYNAMIC:
                case POST_TRANSFER:
                case TRANSFER:
                    intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.IN_BETWEEN);
                    intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                    break;
                case POST_PICKUP:
                case PRE_PICKUP:
                    if (intakeArm.intakeGripState == IntakeArm.GRIP_STATE.OPEN) {
                        intakeOuttakeController.pickupSequence();
                    } else {
                        intakeArm.openGrip();
                        intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                    }
                    break;
                case PICKUP:
                    intakeArm.toggleGrip();
                    if (intakeArm.intakeGripState == IntakeArm.GRIP_STATE.OPEN) {
                        intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                    }
                    break;
            }
        }

        if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.PRE_PICKUP || intakeArm.intakeArmState == IntakeArm.ARM_STATE.PICKUP) {
            GameField.turboFactor = true;
        } else {
            GameField.turboFactor = false;
        }

        if (gp1DpadEnabled) {
            if (!gp1GetStart() && gp1GetSquarePress()) {
                if (intakeArm.intakeArmState != IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER) {
                    intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);
                } else {
                    intakeArm.toggleGrip();
                }
            }
        }

        if (!gp1GetStart()) {
            if ((gp1DpadEnabled && gp1GetDpad_upPress()) || gp1GetLeftBumperPress()) {
                intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
                intakeSlides.moveIntakeSlidesForward();
            }
        } else {
            if (gp1GetDpad_upPress()) {
                gp1DpadEnabled = !gp1DpadEnabled;
            }
        }

        if (gp1DpadEnabled && gp1GetDpad_downPress()) {
            intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.PRE_PICKUP);
            intakeSlides.moveIntakeSlidesBackward();
        }

        if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.PRE_PICKUP
                || intakeArm.intakeArmState == IntakeArm.ARM_STATE.PICKUP ) {
            if (gp1GetRightStickButtonPress()) {
                intakeArm.openGrip();
                intakeArm.toggleSwivel();
            }
            if (gp1DpadEnabled && gp1GetDpad_leftPress()) {
                intakeArm.openGrip();
                intakeArm.moveSwivelLeft();
            }

            if (gp1DpadEnabled && gp1GetDpad_rightPress()) {
                intakeArm.openGrip();
                intakeArm.moveSwivelRight();
            }
        }

        if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.PRE_PICKUP &&
                intakeArm.intakeGripState == IntakeArm.GRIP_STATE.CLOSED) {
            safeWaitMilliSeconds(200);
            intakeArm.senseIntakeSampleColor();
            if (intakeArm.intakeSampleSensed) {
                gp2RumbleFine(200);
                if (intakeOuttakeController.autoTransferEnabled) {
                    intakeOuttakeController.initiateAutoTransfer = true;
                } else {
                    intakeOuttakeController.initiateAutoTransfer = false;
                }
            }
        }
        /*intakeArm.senseIntakeSampleColor();
        if (intakeArm.intakeSampleSensed &&
                intakeArm.intakeArmState == IntakeArm.ARM_STATE.PRE_PICKUP &&
                intakeArm.intakeGripState == IntakeArm.GRIP_STATE.CLOSED) {
            intakeOuttakeController.moveArm(IntakeArm.ARM_STATE.POST_PICKUP);
            gp2RumbleFine(200);
            if (intakeOuttakeController.autoTransferEnabled) {
                intakeOuttakeController.initiateAutoTransfer = true;
            } else {
                intakeOuttakeController.initiateAutoTransfer = false;
            }
        }*/
    }

    public void runOuttake(){

        if (!gp2GetStart()) {
            if ((gp2GetRightStickButtonPress()  || intakeOuttakeController.initiateAutoTransfer)
                    && (intakeArm.intakeArmState == IntakeArm.ARM_STATE.PRE_PICKUP
                            || intakeArm.intakeArmState == IntakeArm.ARM_STATE.POST_PICKUP)) {
                GameField.turboFactor = false;
                //intakeOuttakeController.closeGripAndMoveIntakeArmToPreTransfer();
                intakeOuttakeController.moveOuttakeToTransfer();

                if (outtake.isOuttakeInTransfer()) {
                    if (outtake.lastOuttakeSlideState == Outtake.SLIDE_STATE.HIGH_CHAMBER) {
                        intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakeTransfer();
                        intakeOuttakeController.moveOuttakeArm(Outtake.ARM_STATE.HIGH_CHAMBER);
                    } else {
                        intakeOuttakeController.transferSampleFromIntakePreTransferToOuttakePreDrop();
                    }
                    intakeOuttakeController.moveOuttakeSlidesTo(outtake.lastOuttakeSlideState);
                    /*if (outtake.lastOuttakeSlideState == Outtake.SLIDE_STATE.HIGH_CHAMBER) {
                        outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
                    }*/
                } else {
                    intakeSlides.moveIntakeSlides(IntakeSlides.SLIDES_STATE.TRANSFER_MIN_RETRACTED);
                    intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);
                }

                intakeOuttakeController.initiateAutoTransfer = false;
            }
        } else {
            if (gp2GetRightStickButtonPress()){
                intakeOuttakeController.autoTransferEnabled = !intakeOuttakeController.autoTransferEnabled;
            }
        }

        if(gp2GetDpad_upPress()){
            outtake.lastOuttakeSlideState = Outtake.SLIDE_STATE.HIGH_BUCKET;
            if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.TRANSFER) {
                intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
                safeWaitMilliSeconds(200);
            }
            if (intakeArm.isIntakeArmInSafeStateToMoveOuttake()) {
                if (outtake.outtakeSlidesState != Outtake.SLIDE_STATE.HIGH_BUCKET ) {
                    outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_BUCKET);
                    safeWaitMilliSeconds(300);
                }
                outtake.moveArm(Outtake.ARM_STATE.DROP);
            }
        }

        if(gp2GetDpad_leftPress()){
            outtake.lastOuttakeSlideState = Outtake.SLIDE_STATE.LOW_BUCKET;
            if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.TRANSFER) {
                intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
                safeWaitMilliSeconds(200);
            }
            if (intakeArm.isIntakeArmInSafeStateToMoveOuttake()) {
                if (outtake.outtakeSlidesState != Outtake.SLIDE_STATE.LOW_BUCKET ) {
                    outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.LOW_BUCKET);
                    safeWaitMilliSeconds(300);
                }
                outtake.moveArm(Outtake.ARM_STATE.DROP);
            }
        }

        if(gp2GetLeftBumperPress()){
            /*if (outtake.lastOuttakeSlideState == Outtake.SLIDE_STATE.HIGH_CHAMBER) {
                intakeOuttakeController.moveOuttakeToHighChamber();
                safeWaitMilliSeconds(400);
            }*/

            if (outtake.isOuttakeReadyToDrop()) {
                //outtake.moveWristDrop();
                outtake.openGrip();
                safeWaitMilliSeconds(200);//500
            }

            if (intakeArm.isIntakeArmInSafeStateToMoveOuttake()) {
                outtake.moveArm(Outtake.ARM_STATE.TRANSFER);
                //safeWaitMilliSeconds(200);
                outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
            }
        }

        if(gp2GetTrianglePress()) {
            outtake.lastOuttakeSlideState = Outtake.SLIDE_STATE.HIGH_CHAMBER;
            if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.TRANSFER) {
                intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
                safeWaitMilliSeconds(200);
            }
            if (intakeArm.isIntakeArmInSafeStateToMoveOuttake()) {
                if (outtake.outtakeSlidesState != Outtake.SLIDE_STATE.HIGH_CHAMBER ) {
                    outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.HIGH_CHAMBER);
                    safeWaitMilliSeconds(300);
                }
                outtake.moveArm(Outtake.ARM_STATE.HIGH_CHAMBER);
            }
        }

        if(gp2GetCirclePress()) {
            outtake.ascendToClimb();
        }


        if(gp2GetRightBumperPress()){
            if(outtake.climberAscended){
            outtake.climb();
            }
        }

        if (!gp2GetStart()) {
            if (gp2GetDpad_downPress()) {
                outtake.lastOuttakeSlideState = Outtake.SLIDE_STATE.TRANSFER;
                if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.TRANSFER) {
                    intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.POST_TRANSFER);
                    safeWaitMilliSeconds(200);
                }
                if (intakeArm.isIntakeArmInSafeStateToMoveOuttake()) {
                    if (outtake.outtakeSlidesState != Outtake.SLIDE_STATE.TRANSFER ) {
                        outtake.moveOuttakeSlides(Outtake.SLIDE_STATE.TRANSFER);
                        safeWaitMilliSeconds(300);
                    }
                    outtake.moveArm(Outtake.ARM_STATE.DROP);
                }
            }
        } else {
            if (gp2GetDpad_downPress()) {
                if (intakeArm.intakeArmState == IntakeArm.ARM_STATE.TRANSFER) {
                    intakeOuttakeController.moveIntakeArm(IntakeArm.ARM_STATE.EJECT_OR_PRE_TRANSFER);
                }
                outtake.manualResetOuttakeMotor();
            }
        }

        if (outtake.outtakeSlidesState == Outtake.SLIDE_STATE.TRANSFER) {
            if (!outtake.outtakeStallTimingFlag) {
                outtake.outtakeStallTimingFlag = true;
                outtake.outtakeStallTimer.reset();
            } else {
                if (outtake.outtakeStallTimer.time() > outtake.STALL_TIME) {
                    outtake.resetOuttakeMotorMode();
                    outtake.outtakeStallTimingFlag = false;
                }
            }
        } else {
            outtake.outtakeStallTimingFlag = false;
        }
    }

    //*********** KEY PAD MODIFIERS BELOW ***********

    //**** Gamepad buttons

    //Records last button press to deal with single button presses doing a certain methods
    boolean gp1ButtonALast = false;
    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;
    boolean gp1RightBumperLast = false;
    boolean gp1LeftBumperLast = false;
    boolean gp1Dpad_upLast = false;
    boolean gp1Dpad_downLast = false;
    boolean gp1Dpad_leftLast = false;
    boolean gp1Dpad_rightLast = false;
    boolean gp1LeftTriggerLast = false;
    boolean gp1RightTriggerLast = false;
    boolean gp1LeftStickButtonLast = false;
    boolean gp1RightStickButtonLast = false;

    boolean gp2ButtonALast = false;
    boolean gp2ButtonBLast = false;
    boolean gp2ButtonXLast = false;
    boolean gp2ButtonYLast = false;
    boolean gp2RightBumperLast = false;
    boolean gp2LeftBumperLast = false;
    boolean gp2Dpad_upLast = false;
    boolean gp2Dpad_downLast = false;
    boolean gp2Dpad_leftLast = false;
    boolean gp2Dpad_rightLast = false;
    boolean gp2LeftTriggerLast = false;
    boolean gp2RightTriggerLast = false;
    boolean gp2LeftStickButtonLast = false;
    boolean gp2RightStickButtonLast = false;


    /**
     * Method to convert linear map from gamepad1 and gamepad2 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput)/1.5; //0.25
    }

    /**
     * Method to implement turbo speed mode - from reduced speed of 25% of cubic factor to
     * 100% speed, but controlled by acceleration of the force of pressing the Right Tigger.
     *
     * @param stickInput input value of button stick vector
     * @return modified value of button stick vector
     */
    public double gp1TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp1GetRightTrigger();
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        //acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        //acceleration_factor = 1.0 + 1.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 0.5 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }

    public double gp2TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp2GetRightTrigger();
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        //acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 1.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }

    /**
     * Methods to get the value of gamepad Left stick X for Pan motion X direction.
     * This is the method to apply any directional modifiers to match to the X plane of robot.
     * No modifier needed for Hazmat Freight Frenzy Robot.
     *
     * @return gpGamepad1.left_stick_x
     */
    public double gp1GetLeftStickX() {
        return hzGamepad1.left_stick_x;
    }

    public double gp2GetLeftStickX() {
        return hzGamepad2.left_stick_x;
    }

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Freight Frenzy Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y
     */
    public double gp1GetLeftStickY() { return hzGamepad1.left_stick_y; }

    public double gp2GetLeftStickY() { return hzGamepad2.left_stick_y; }

    /**
     * Methods to get the value of gamepad Right stick X to keep turning.
     * This is the method to apply any directional modifiers to match to the turn direction robot.
     * No modifier needed for Hazmat Freight Frenzy Robot.
     *
     * @return gpGamepad1.right_stick_x
     */
    public double gp1GetRightStickX() {
        return hzGamepad1.right_stick_x;
    }

    public double gp2GetRightStickX() {
        return hzGamepad2.right_stick_x;
    }

    public double gp1GetRightStickY() {
        return hzGamepad1.right_stick_y;
    }

    public double gp2GetRightStickY() {
        return hzGamepad2.right_stick_y;
    }

    /**
     * Methods to get the value of gamepad Right Trigger for turbo mode (max speed).
     * This is the method to apply any modifiers to match to action of turbo mode for each driver preference.
     * For Hazmat Freight Frenzy Right Trigger pressed means turbo mode on.
     *
     * @return gpGamepad1.right_trigger
     * @return gpGamepad2.right_trigger
     */
    public double gp1GetRightTrigger() {
        return hzGamepad1.right_trigger;
    }
    public double gp2GetRightTrigger() {
        return hzGamepad2.right_trigger;
    }

    /**
     * gp1 right trigger press cubic value when pressed
     * @return
     */
    public boolean gp1GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp1RightTriggerLast && (gp1GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp1RightTriggerLast = (gp1GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    public boolean gp2GetRightTriggerPersistent() {
        if (gp2GetRightTrigger()>0.7) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * gp2 right trigger press cubic value when pressed
     * @return
     */
    public boolean gp2GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp2RightTriggerLast && (gp2GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp2RightTriggerLast = (gp2GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Trigger
     *
     * @return gpGamepad1.left_trigger
     * @return gpGamepad2.left_trigger
     */
    public double gp1GetLeftTrigger() {
        return hzGamepad1.left_trigger;
    }
    public double gp2GetLeftTrigger() {
        return hzGamepad2.left_trigger;
    }

    /**
     * The range of the gp1 left trigger cubic press
     * @return
     */
    public boolean gp1GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp1LeftTriggerLast && (gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp1LeftTriggerLast = (gp1GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    public boolean gp1GetLeftTriggerPersistent() {
        boolean isPressedLeftTrigger = false;
        if ((gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        return isPressedLeftTrigger;
    }

    public boolean gp2GetLeftTriggerPersistent() {
        boolean isPressedLeftTrigger = false;
        if ((gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        return isPressedLeftTrigger;
    }

    /**
     * The range of the gp2 left trigger cubic press values
     * @return
     */
    public boolean gp2GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp2LeftTriggerLast && (gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp2LeftTriggerLast = (gp2GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Bumper
     *
     * @return gpGamepad1.left_bumper
     * @return gpGamepad2.left_bumper
     */
    public boolean gp1GetLeftBumper() {
        return hzGamepad1.left_bumper;
    }

    public boolean gp2GetLeftBumper() {
        return hzGamepad2.left_bumper;
    }

    /**
     * Method to track if Left Bumper was pressed
     * To ensure that the continuous holding of the left bumper does not cause a contiual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing hold or release of button should not trigger action.
     *
     * @return isPressedLeftBumper| = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp1LeftBumperLast && hzGamepad1.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp1LeftBumperLast = hzGamepad1.left_bumper;
        return isPressedLeftBumper;
    }

    public boolean gp2GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp2LeftBumperLast && hzGamepad2.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp2LeftBumperLast = hzGamepad2.left_bumper;
        return isPressedLeftBumper;
    }

    /**
     * Methods to get the value of gamepad Right Bumper
     *
     * @return gpGamepad1.right_bumper
     * @return gpGamepad2.right_bumper
     */
    public boolean gp1GetRightBumper() {
        return hzGamepad1.right_bumper;
    }

    public boolean gp2GetRightBumper() {
        return hzGamepad2.right_bumper;
    }
    /**
     * Method to track if Right Bumper was pressed
     * To ensure that the continuous holding of the right bumper does not cause a continual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedRightBumper = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp1RightBumperLast && hzGamepad1.right_bumper) {
            isPressedRightBumper = true;
        }
        gp1RightBumperLast = hzGamepad1.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp2GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp2RightBumperLast && hzGamepad2.right_bumper) {
            isPressedRightBumper = true;
        }
        gp2RightBumperLast = hzGamepad2.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp1GetRightBumperPersistant(){
        return hzGamepad1.right_bumper;
    }
    public boolean gp2GetRightBumperPersistant(){
        return hzGamepad2.right_bumper;
    }

    /**
     * Method to track if Button A was pressed
     * To ensure that the continuous holding of Button A does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButton A = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && hzGamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = hzGamepad1.a;
        return isPressedButtonA;
    }

    public boolean gp2GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp2ButtonALast && hzGamepad2.a) {
            isPressedButtonA = true;
        }
        gp2ButtonALast = hzGamepad2.a;
        return isPressedButtonA;
    }

    public boolean gp1GetA(){
        return hzGamepad1.a;
    }

    public boolean gp2GetA(){
        return hzGamepad2.a;
    }


    /**
     * Method to track if Button Y was pressed
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonY = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && hzGamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = hzGamepad1.y;
        return isPressedButtonY;
    }

    public boolean gp2GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp2ButtonYLast && hzGamepad2.y) {
            isPressedButtonY = true;
        }
        gp2ButtonYLast = hzGamepad2.y;
        return isPressedButtonY;
    }
    public boolean gp1GetY(){
        return hzGamepad1.y;
    }
    public boolean gp2GetY(){
        return hzGamepad2.y;
    }

    /**
     * Method to track if Button X was pressed
     * To ensure that the continuous holding of Button X does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonX = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp1ButtonXLast && hzGamepad1.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = hzGamepad1.x;
        return isPressedButtonX;
    }

    public boolean gp2GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp2ButtonXLast && hzGamepad2.x) {
            isPressedButtonX = true;
        }
        gp2ButtonXLast = hzGamepad2.x;
        return isPressedButtonX;
    }
    public boolean gp1GetX(){
        return hzGamepad1.x;
    }
    public boolean gp2GetX(){
        return hzGamepad2.x;
    }

    /**
     * Method to track if Button B was pressed to move Arm
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonB = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp1ButtonBLast && hzGamepad1.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = hzGamepad1.b;
        return isPressedButtonB;
    }
    public boolean gp2GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp2ButtonBLast && hzGamepad2.b) {
            isPressedButtonB = true;
        }
        gp2ButtonBLast = hzGamepad2.b;
        return isPressedButtonB;
    }
    public boolean gp1GetB(){
        return hzGamepad1.b;
    }
    public boolean gp2GetB(){
        return hzGamepad2.b;
    }

    //Map to PS4 Gamepad buttons
    public boolean gp1GetSquarePress(){ return gp1GetButtonXPress();}
    public boolean gp2GetSquarePress(){ return gp2GetButtonXPress();}
    public boolean gp1GetSquarePersistent(){ return gp1GetX();}
    public boolean gp2GetSquarePersistent(){ return gp2GetX();}
    public boolean gp1GetTrianglePress(){ return gp1GetButtonYPress();}
    public boolean gp2GetTrianglePress(){ return gp2GetButtonYPress();}
    public boolean gp1GetTrianglePersistent(){ return gp1GetY();}
    public boolean gp2GetTrianglePersistent(){ return gp2GetY();}
    public boolean gp1GetCrossPress(){ return gp1GetButtonAPress();}
    public boolean gp2GetCrossPress(){ return gp2GetButtonAPress();}
    public boolean gp1GetCrossPersistent(){ return gp1GetA();}
    public boolean gp2GetCrossPersistent(){ return gp2GetA();}
    public boolean gp1GetCirclePress(){ return gp1GetButtonBPress();}
    public boolean gp2GetCirclePress(){ return gp2GetButtonBPress();}
    public boolean gp1GetCirclePersistent(){ return gp1GetB();}
    public boolean gp2GetCirclePersistent(){ return gp2GetB();}



    /**
     * Method to track if Dpad_up was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_up = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp1Dpad_upLast && hzGamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast = hzGamepad1.dpad_up;
        return isPressedDpad_up;
    }
    public boolean gp2GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp2Dpad_upLast && hzGamepad2.dpad_up) {
            isPressedDpad_up = true;
        }
        gp2Dpad_upLast = hzGamepad2.dpad_up;
        return isPressedDpad_up;
    }

    public boolean gp1GetDpad_up(){
        return hzGamepad1.dpad_up;
    }
    public boolean gp2GetDpad_up(){
        return hzGamepad2.dpad_up;
    }

    /**
     * Method to track if Dpad_down was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_down = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp1Dpad_downLast && hzGamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = hzGamepad1.dpad_down;
        return isPressedDpad_down;
    }
    public boolean gp2GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp2Dpad_downLast && hzGamepad2.dpad_down) {
            isPressedDpad_down = true;
        }
        gp2Dpad_downLast = hzGamepad2.dpad_down;
        return isPressedDpad_down;
    }

    public boolean gp1GetDpad_down(){
        return hzGamepad1.dpad_down;
    }
    public boolean gp2GetDpad_down(){
        return hzGamepad2.dpad_down;
    }

    /**
     * Method to track if Dpad_left was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp1Dpad_leftLast && hzGamepad1.dpad_left) {
            isPressedDpad_left = true;
        }
        gp1Dpad_leftLast = hzGamepad1.dpad_left;
        return isPressedDpad_left;
    }
    public boolean gp2GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp2Dpad_leftLast && hzGamepad2.dpad_left) {
            isPressedDpad_left = true;
        }
        gp2Dpad_leftLast = hzGamepad2.dpad_left;
        return isPressedDpad_left;
    }

    public boolean gp1GetDpad_left(){
        return hzGamepad1.dpad_left;
    }
    public boolean gp2GetDpad_left(){
        return hzGamepad2.dpad_left;
    }

    /**
     * Method to track if Dpad_right was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp1Dpad_rightLast && hzGamepad1.dpad_right) {
            isPressedDpad_right = true;
        }
        gp1Dpad_rightLast = hzGamepad1.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp2GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp2Dpad_rightLast && hzGamepad2.dpad_right) {
            isPressedDpad_right = true;
        }
        gp2Dpad_rightLast = hzGamepad2.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp1GetDpad_right(){
        return hzGamepad1.dpad_right;
    }
    public boolean gp2GetDpad_right(){
        return hzGamepad2.dpad_right;
    }

    public boolean gp1GetStart(){
        return hzGamepad1.start;
    }
    public boolean gp2GetStart(){
        return hzGamepad2.start;
    }

    public boolean gp1GetRightStickButtonPress() {
        boolean isPressedRightStickButton = false;
        if (!gp1RightStickButtonLast && hzGamepad1.right_stick_button) {
            isPressedRightStickButton = true;
        }
        gp1RightStickButtonLast = hzGamepad1.right_stick_button;
        return isPressedRightStickButton;
    }

    public boolean gp1GetRightStickButtonPersistant(){
        return hzGamepad1.right_stick_button;
    }

    public boolean gp1GetLeftStickButtonPress() {
        boolean isPressedLeftStickButton = false;
        if (!gp1LeftStickButtonLast && hzGamepad1.left_stick_button) {
            isPressedLeftStickButton = true;
        }
        gp1LeftStickButtonLast = hzGamepad1.left_stick_button;
        return isPressedLeftStickButton;
    }

    public boolean gp1GetLeftStickButtonPersistant(){
        return hzGamepad1.left_stick_button;
    }

    public boolean gp2GetRightStickButtonPress() {
        boolean isPressedRightStickButton = false;
        if (!gp2RightStickButtonLast && hzGamepad2.right_stick_button) {
            isPressedRightStickButton = true;
        }
        gp2RightStickButtonLast = hzGamepad2.right_stick_button;
        return isPressedRightStickButton;
    }

    public boolean gp2GetRightStickButtonPersistant(){
        return hzGamepad2.right_stick_button;
    }

    public boolean gp2GetLeftStickButtonPress() {
        boolean isPressedLeftStickButton = false;
        if (!gp2LeftStickButtonLast && hzGamepad2.left_stick_button) {
            isPressedLeftStickButton = true;
        }
        gp2LeftStickButtonLast = hzGamepad2.left_stick_button;
        return isPressedLeftStickButton;
    }

    public boolean gp2GetLeftStickButtonPersistant(){
        return hzGamepad2.left_stick_button;
    }

    public void gp1RumbleFine(int duration){
        hzGamepad1.rumble(0,1, duration);
    }

    public void gp2RumbleFine(int duration){
        hzGamepad2.rumble(0,1, duration);
    }

}