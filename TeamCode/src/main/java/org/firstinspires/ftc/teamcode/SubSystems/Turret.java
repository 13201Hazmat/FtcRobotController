package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runIntakeMotor checks if the motor is not running and runs the intake  <BR>
 *     stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
 */

public class Turret {

    //Initialization of <Fill>
    public DcMotor turretmotor;

    //Initialization of <Fill>
    public enum TURRET_MOTOR_POSITION{

    }

    //Initialization of <Fill>
    public TURRET_MOTOR_POSITION turretMotorPosition;

    //Initialization of <Fill>
    public double turretMotorPower = 1.0;

    //Constructor
    public Turret(HardwareMap hardwareMap){
        initTurret();
    }

    //Method is able to <Fill>
    public void initTurret(){

    }
}