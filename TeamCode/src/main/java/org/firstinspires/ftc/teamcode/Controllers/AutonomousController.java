package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;
import org.firstinspires.ftc.teamcode.SubSystems.SubsystemTemplate;

/**
 * Defenition of the AutoControl Class <BR>
 *
 * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
 * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
 *
 */

public class AutonomousController {

    //Create gamepad object reference to connect to gamepad1
    public DriveTrain driveTrain;
    public Intake intake;
    public Elevator elevator;
    public Magazine magazine;
    public Spinner spinner;
    public MajorArm majorArm;

    public Pose2d startPose = GameField.BLUE_STARTPOS_1;

    // TODO: Declare autonomous option logic based on key pad selection
    /* Example
    public boolean launchHighGoalOrPowerShot = false;
    public boolean dropFirstWobbleGoal = false;
    public boolean pickRingFromTargetMarker = false;
    public boolean launchRingsPickedFromTargetMarkerToHighGoal = false;
    public boolean pickAndDropSecondWobbleGoal = false;
         */

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * TODO: Add more subsystems in declaration
     */
    public AutonomousController(DriveTrain driveTrain,
                                Intake intake,
                                Elevator elevator,
                                Magazine magazine,
                                Spinner spinner,
                                MajorArm majorArm) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.elevator = elevator;
        this.magazine = magazine;
        this.spinner = spinner;
        this.majorArm = majorArm;
    }

    /**
     * Main control function that calls the runs of each of the subsystems in sequence
     */
    public void runAutoControl(){
        int counter = 0;
        while (counter < 5) {
            runAutoIntake();
            runAutoElevator();
            runAutoMagazine();
            runAutoSpinner();
            runAutoMajorArm();
            counter++;
        }
    }



    // Define and delcare autonomous states
    // Example
    enum AUTO_SUBSYSTEM1_STATE{
        START,
        STOP,
        //TODO:Update Subsystem states as appropriate
    }
    AUTO_SUBSYSTEM1_STATE autoSubsystem1State = AUTO_SUBSYSTEM1_STATE.STOP;

    /**
     * set Launch Target PowerShot2 state
     */
    public void setSubsystem1ToState(){
        autoSubsystem1State = AUTO_SUBSYSTEM1_STATE.START;
        runAutoControl();
    }

    /**
     * run Intake Control State machine response
     * Also deactivate Launch readiness when Intake is started
     */
    public void runSubsystem1Control(){

        if (autoSubsystem1State == AUTO_SUBSYSTEM1_STATE.START){
            //TODO: Add state setting code for Subsystem1
            /* Set state for subsystem - Example
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
             */
        }

        if (autoSubsystem1State == AUTO_SUBSYSTEM1_STATE.STOP){
            //TODO: Add state setting code for Subsystem1
             /* Set state for subsystem - Example
            acHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
              */
        }
    }


    //TODO: Add more run Subsystem Control functions

    /**
     * Intake Commands :
     *      startAutoIntake()
     *      stopAutoIntake()
     */

    enum AUTO_INTAKE_STATE{
        RUNNING,
        STOPPED,
    }
    AUTO_INTAKE_STATE autoIntakeState = AUTO_INTAKE_STATE.STOPPED;

    public void startAutoIntake(){
        autoIntakeState = AUTO_INTAKE_STATE.RUNNING;
        runAutoControl();
    }

    public void stopAutoIntake(){
        autoIntakeState = AUTO_INTAKE_STATE.STOPPED;
        runAutoControl();
    }

    public void runAutoIntake() {
        if (autoIntakeState == AUTO_INTAKE_STATE.RUNNING && elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0) {
            if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.COLLECT) {
                magazine.moveMagazineToCollect();
            }
            intake.startForwardIntakeMotor();
        } else { //(autoIntakeState == AUTO_INTAKE_STATE.STOPPED)
            intake.stopIntakeMotor();
        }
    }

    /**
     * Elevator Commands :
     *      moveAutoElevatorLevel0()
     *      moveAutoElevatorLevel1()
     *      moveAutoElevatorLevel2()
     *      moveAutoElevatorLevel3()
     */

    enum  AUTO_ELEVATOR_STATE{
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }
    //TODO: Add state variable and initializes

    public void moveAutoElevatorLevel0(){
        //TODO : Add code
    }
    //TODO : Add other Levels

    public void runAutoElevator() {
        //TODO : Add code
    }

    /**
     * Magazine Commands :
     *      moveAutoMagazineToCollect()
     *      moveAutoMagazineToTransport()
     *      moveAutoMagazineToDrop()
     */
    //TODO: Add command methods
    enum AUTO_MAGAZINE_STATE{
        COLLECT,
        TRANSPORT,
        DROP,
    }
    //TODO: Add state variable and initializs

    public void runAutoMagazine() {
        //TODO : Add code
    }

    /**
     * Major Arm Commands :
     */

    enum AUTO_MAJOR_ARM_STATE{
        PICKUP,
        PARKED,
    }
    AUTO_MAJOR_ARM_STATE autoMajorArmState = AUTO_MAJOR_ARM_STATE.PARKED;

    public void pickupAutoMajorArm(){
        autoMajorArmState = AUTO_MAJOR_ARM_STATE.PICKUP;
        runAutoControl();
    }

    public void parkAutoMajorArm(){
        autoMajorArmState = AUTO_MAJOR_ARM_STATE.PARKED;
        runAutoControl();
    }
    public enum AUTO_MAJOR_CLAW_STATE {
        OPEN,
        CLOSED,
    }
    AUTO_MAJOR_CLAW_STATE autoMajorClawState = AUTO_MAJOR_CLAW_STATE.CLOSED;

    public void openAutoMajorClaw(){
        autoMajorClawState = AUTO_MAJOR_CLAW_STATE.OPEN;
        runAutoControl();
    }

    public void closeAutoMajorClaw(){
        autoMajorClawState = AUTO_MAJOR_CLAW_STATE.CLOSED;
        runAutoControl();
    }

    public void runAutoMajorArm() {
        if(autoMajorArmState == AUTO_MAJOR_ARM_STATE.PARKED){
            majorArm.moveArmParkingPosition();
        }
        else{
            majorArm.moveArmPickupPosition();
        }

        if(autoMajorClawState == AUTO_MAJOR_CLAW_STATE.OPEN){
            majorArm.openClaw();
        }
        else{
            majorArm.closeClaw();
        }
    }

    /**
     * Spinner Commands :
     */

    public enum AUTO_SPINNER_STATE{
        CLOCKWISE,
        ANTICLOCKWISE,
        STOPPED,
    }

    AUTO_SPINNER_STATE autoSpinnerState = AUTO_SPINNER_STATE.STOPPED;

    public void startAutoSpinnerCW() {
    autoSpinnerState = AUTO_SPINNER_STATE.CLOCKWISE;
    runAutoControl();
    }

    public void startAutoSpinnerCCW() {
        autoSpinnerState = AUTO_SPINNER_STATE.ANTICLOCKWISE;
        runAutoControl();
    }

    public void stopAutoSpinner(){
        autoSpinnerState = AUTO_SPINNER_STATE.STOPPED;
        runAutoControl();
    }

    public void runAutoSpinner() {
        if(autoSpinnerState == AUTO_SPINNER_STATE.CLOCKWISE){
            spinner.runSpinnerMotorClockwise();
        }
        else if(autoSpinnerState == AUTO_SPINNER_STATE.ANTICLOCKWISE){
            spinner.runSpinnerMotorAnticlockwise();
        }
        else{
            spinner.stopSpinnerMotor();
        }
    }

}