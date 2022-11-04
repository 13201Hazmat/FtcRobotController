package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Definition of Arm Class <BR>
 *
 * Arm consists of linear slides starting from the shoulder(pivot) and can extend to a fixed length
 * by the driver, holding the hand at the end<BR>
 *
 * The states are as followed: <BR>
 *     ARM_STATE - example if linear slides are either fully extended or fully retracted  <BR>
 *     ARM_MOTOR_POSITION -  linear slides are at preset positions <BR>
 *
 * The functions are as followed: <BR>
 *     initArm resets the motors and positions <BR>
 *     turnArmBrakeModeOn and turnArmBrakeModeOff puts the arm motor in a stopped or active state <BR>
 *     moveToJunction functions set the target position to preset positions corresponding to function<BR>
 *     extendArm and retractArm functions extend and retract arm based on a delta value determined
 *     by the joystick <BR>
 *     resetArm resets the arm to the original position and states <BR>
 *     runArmToLevel runs the arm to the levels determined by the other functions <BR>
 */

public class Arm {
    //Initialization of armmotor
    public DcMotorEx armmotor;

    //Arm states, either fully extended or retracted all the way

    //Initialization of ARM_MOTOR_POSITION
    public enum ARM_MOTOR_STATE {
        PARKED,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        MAX_EXTENDED

    }

    //Initialization of ARM_MOTOR_POSITION and ARM_STATE enums
    public ARM_MOTOR_STATE armMotorState;



    //Constants for Arm positions
    public static final int PICKUP_POSITION = 0; //Need tested values
    public static final int GROUND_JUNCTION_POSITION = 0; //Need tested values
    public static final int LOW_JUNCTION_POSITION = (int) (537* 1.25); //Need tested values
    public static final int MEDIUM_JUNCTION_POSITION = (int) (537* 2.5); //Need tested values
    public static final int HIGH_JUNCTION_POSITION = (int) (537* 3.75); //Need tested values
    public static final int MAX_EXTENDED_POSITION = (int) (537* 5); //Need tested value

    public static final int AUTO_RETRACTION_DELTA_POSITION = 200; //need tested values
    public static final int ARM_DELTA_COUNT_MAX = 200; //need tested values

    //Different constants of arm speed
    public double HIGH_POWER = 1.0;
    public double MED_POWER = 0.5;
    public double LOW_POWER = 0.2;

    public int armDeltaCount = 0; //Need tested value

    public boolean runShoulderToLevelState = false;

    public int armCurrentArmPositionCount = PICKUP_POSITION; //Default arm position count

    //Constructor`
    public Arm(HardwareMap hardwareMap){
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        initArm();
    }

    //Method is able to initialize the arm
    public void initArm(){
        resetArm();
        turnArmBrakeModeOff();
        armmotor.setPositionPIDFCoefficients(5.0);
        armmotor.setTargetPosition(PICKUP_POSITION);
        armmotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    //Turns on the brake for arm motor
    public void turnArmBrakeModeOn(){
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns brake for arm motor off
    public void turnArmBrakeModeOff() {
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets arm position to ground junction
    public void moveToArmGroundJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(GROUND_JUNCTION_POSITION);
        runShoulderToLevelState = true;

    }

    //Sets arm position to low junction
    public void moveToArmLowJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(LOW_JUNCTION_POSITION);
        runShoulderToLevelState = true;
    }

    //Sets arm position or mid junction
    public void moveToArmMidJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(MEDIUM_JUNCTION_POSITION);
        runShoulderToLevelState = true;
    }

    //Sets arm position to high junction
    public void moveToArmHighJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(HIGH_JUNCTION_POSITION);
        runShoulderToLevelState = true;
    }



    //retracts the arm for joystick control
    public void retractArm(double joystickAmount){
        armDeltaCount = (int) (Math.pow((joystickAmount * 1.25 - 0.25), 3) * ARM_DELTA_COUNT_MAX); //Function is normalized 0.2-1 to 0-1
        if (armCurrentArmPositionCount > PICKUP_POSITION + armDeltaCount ){
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = armCurrentArmPositionCount - ARM_DELTA_COUNT_MAX;
            armmotor.setTargetPosition(armCurrentArmPositionCount);
            runShoulderToLevelState = true;
        }
    }

    //extends the arm for the joystick control
    public void extendArm( /* getShoulderPositionCount */ double joystickAmount ){
        //TODO - Convert MAX_EXTENDED to a varible value when shoulder angle < 0, use auto retraction
        armDeltaCount = (int) (Math.pow((joystickAmount * 1.25 - 0.25), 3) * ARM_DELTA_COUNT_MAX); //Function is normalized 0.2-1 to 0-1
        //maxExtended = (ROBOT_HEIGHT - 2)/Math.cos(getShoulderPositionCount * CONVERSION_FACTOR_TO_DEGREES) - F; Algorithm to not hit the ground
        if (armCurrentArmPositionCount < MAX_EXTENDED_POSITION - armDeltaCount){
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = armCurrentArmPositionCount + ARM_DELTA_COUNT_MAX;

        } else{
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = MAX_EXTENDED_POSITION;
        }
        armmotor.setTargetPosition(armCurrentArmPositionCount);
        runShoulderToLevelState = true;
    }

    //sets the arm motor power
    public void runShoulderToLevel(double power){
        armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runShoulderToLevelState == true){
            armmotor.setPower(power);
            runShoulderToLevelState = false;
        } else{
            armmotor.setPower(0.0);
        }
    }

    //Resets the arm
    public void resetArm(){
        DcMotorEx.RunMode runMode = armmotor.getMode();
        armmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setMode(runMode);
    }

    //Returns the current arm position
    public int getArmPositionCount(){
        return armmotor.getCurrentPosition();
    }
}





