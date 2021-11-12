package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;

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

    public Pose2d startPose = GameField.BLUE_WAREHOUSE_STARTPOS;

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
        if (autoIntakeState == AUTO_INTAKE_STATE.RUNNING &&
                elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0) {
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
    AUTO_ELEVATOR_STATE autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_0;

    public void moveAutoElevatorLevel0(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_0;
        runAutoControl();
    }

    public void moveAutoElevatorLevel1(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_1;
        runAutoControl();
    }

    public void moveAutoElevatorLevel2(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_2;
        runAutoControl();
    }

    public void moveAutoElevatorLevel3(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_3;
        runAutoControl();
    }

    public void runAutoElevator() {
        if (elevator.elevatorState == Elevator.ELEVATOR_STATE.LEVEL_0){
            elevator.moveElevatorLevel0Position();
            autoMagazineState = AUTO_MAGAZINE_STATE.COLLECT;

        } else {
            autoIntakeState = AUTO_INTAKE_STATE.STOPPED;
            autoMagazineState = AUTO_MAGAZINE_STATE.TRANSPORT;
            switch (elevator.elevatorState){
                case LEVEL_1:
                    elevator.moveElevatorLevel1Position();
                    break;
                case LEVEL_2:
                    elevator.moveElevatorLevel2Position();
                    break;
                case LEVEL_3:
                    elevator.moveElevatorLevel3Position();
                    break;
            }
        }
    }

    /**
     * Magazine Commands :
     *      moveAutoMagazineToCollect()
     *      moveAutoMagazineToTransport()
     *      moveAutoMagazineToDrop()
     */
    enum AUTO_MAGAZINE_STATE{
        COLLECT,
        TRANSPORT,
        DROP,
    }
    AUTO_MAGAZINE_STATE autoMagazineState = AUTO_MAGAZINE_STATE.COLLECT;

    public void moveAutoMagazineToCollect(){
        autoMagazineState = AUTO_MAGAZINE_STATE.COLLECT;
        runAutoControl();
    }

    public void moveAutoMagazineToTransport(){
        autoMagazineState = AUTO_MAGAZINE_STATE.TRANSPORT;
        runAutoControl();
    }

    public void moveAutoMagazineToDrop(){
        autoMagazineState = AUTO_MAGAZINE_STATE.DROP;
        runAutoControl();
    }

    public void runAutoMagazine() {
        switch (autoMagazineState){
            case TRANSPORT:
                magazine.moveMagazineToTransport();
                break;
            case DROP :
                magazine.moveMagazineToDrop();
                break;
            case COLLECT:
                magazine.moveMagazineToCollect();
                break;
        }
    }

    /**
     * Major Arm Commands :
     */

    enum AUTO_MAJOR_ARM_STATE{
        PICKUP,
        PARKED,
    }
    AUTO_MAJOR_ARM_STATE autoMajorArmState = AUTO_MAJOR_ARM_STATE.PARKED;

    public void moveAutoMajorArmPickup(){
        autoMajorArmState = AUTO_MAJOR_ARM_STATE.PICKUP;
        runAutoControl();
    }

    public void moveAutoMajorArmPark(){
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
    /**
     * Hybrid Commands For Autonomous OpMode
     */
    public void runAutoPickUpCapstone() {
        majorArm.moveArmPickupPosition();
        majorArm.openClaw();
        //add the spline function
        majorArm.closeClaw();
        majorArm.moveArmParkingPosition();
    }


}