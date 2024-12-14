package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {
    public DcMotorEx climberStg1Motor = null;
    public CRServo climberStg1ServoLeft, climberStg1ServoRight;

    public enum SERVO_STATE {
        INITIAL,
        ASCENDED
    }
    public SERVO_STATE climberServoState = SERVO_STATE.INITIAL;

    public double climberServoPower = 1.0;
    public int CLIMBER_SERVO_ASCEND_TIME = 1500;//1250;
    public int CLIMBER_SERVO_DESCEND_TIME = 200;
    public ElapsedTime climberServoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public double climberServoPowerWhenClimbing = 0.8;

    public boolean climberActivated = false;

    public boolean climbingStg1Started = false;

    public enum STAGE1_MOTOR_STATE {
        INITIAL(0), //Position
        CLIMBED(1300), //2000, 60 rpm motor
        MAX(2500); //9000

        public final int motorPosition;
        STAGE1_MOTOR_STATE(int motorPosition) {
            this.motorPosition = motorPosition;
        }
    }
    public STAGE1_MOTOR_STATE climberStg1MotorState = STAGE1_MOTOR_STATE.INITIAL;

    public double climberMotorStg1CurrentPosition = climberStg1MotorState.motorPosition;
    public double climberMotorStg1NewPosition = climberStg1MotorState.motorPosition;

    public boolean runClimberMotorToLevelState = false;

    public double startCurrentPosition;

    public Telemetry telemetry;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberStg1Motor = hardwareMap.get(DcMotorEx.class, "climber1_motor");
        climberStg1ServoLeft = hardwareMap.get(CRServo.class, "climber1_servo_left");
        climberStg1ServoRight = hardwareMap.get(CRServo.class, "climber1_servo_right");
        climberStg1ServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        initClimber();
    }

    public void initClimber(){
        climberStg1Motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberStg1Motor.setPositionPIDFCoefficients(10.0); //5
        climberStg1Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        startCurrentPosition = climberStg1Motor.getCurrentPosition();
        turnClimberStg1BrakeModeOff();
    }

    public void ascendClimberStg1Servo(){
        climberServoTimer.reset();
        climberStg1ServoLeft.setPower(climberServoPower);
        climberStg1ServoRight.setPower(climberServoPower);
        while (climberServoTimer.time() < CLIMBER_SERVO_ASCEND_TIME) {
            //TODO : PASS OPMODE TO ALL SUBSYSTEMS and ENSURE SAFE WHILE LOOPS
        }
        climberStg1ServoLeft.setPower(0);
        climberStg1ServoRight.setPower(0);
        climberServoState = SERVO_STATE.ASCENDED;

    }

    public void descendClimberStg1Servo(){
        climberServoTimer.reset();
        climberStg1ServoLeft.setPower(-climberServoPower/8);
        climberStg1ServoRight.setPower(-climberServoPower/8);
        while (climberServoTimer.time() < CLIMBER_SERVO_DESCEND_TIME) {
            //TODO : PASS OPMODE TO ALL SUBSYSTEMS and ENSURE SAFE WHILE LOOPS
        }
        climberStg1ServoLeft.setPower(0);
        climberStg1ServoRight.setPower(0);
    }

    public void turnClimberStg1BrakeModeOn(){
        climberStg1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnClimberStg1BrakeModeOff(){
        climberStg1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void moveClimberStg1Motor(STAGE1_MOTOR_STATE toClimberMotorState){
        turnClimberStg1BrakeModeOn();
        climberMotorStg1CurrentPosition = climberStg1Motor.getCurrentPosition();
        climberStg1Motor.setTargetPosition((int)(toClimberMotorState.motorPosition + startCurrentPosition));
        climberStg1MotorState = toClimberMotorState;
        runClimberMotorToLevelState = true;
        runClimberStg1MotorToLevel();
    }

    //sets the ClimberStage1 motor power
    public void runClimberStg1MotorToLevel(){
        double power = 1.0;

        if (climberStg1MotorState == STAGE1_MOTOR_STATE.INITIAL) {
            turnClimberStg1BrakeModeOff();
        } else {
            turnClimberStg1BrakeModeOn();
        }

        climberStg1Motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runClimberMotorToLevelState){
            climberStg1Motor.setPower(power);
            /*TODO if (!outtakeMotor.isBusy()) */runClimberMotorToLevelState = false;
        } else{
            climberStg1Motor.setPower(0.0);
        }
    }

    public void moveClimberStg1UpInSteps(double power){
        climberStg1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg1BrakeModeOn();

        int climberMotorCurrentPosition = climberStg1Motor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < STAGE1_MOTOR_STATE.MAX.motorPosition)){
            climberStg1Motor.setTargetPosition(climberMotorCurrentPosition + STAGE1_MOTOR_STATE.CLIMBED.motorPosition);
            climberStg1Motor.setPositionPIDFCoefficients(10.0);
            climberStg1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberStg1Motor.setPower(power);
        } else {
            climberStg1Motor.setPower(0);
        }
    }

    public void moveClimberStg1DownInSteps(double power){
        climberStg1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg1BrakeModeOn();

        int climberMotorCurrentPosition = climberStg1Motor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition > STAGE1_MOTOR_STATE.INITIAL.motorPosition)){
            climberStg1Motor.setTargetPosition(climberMotorCurrentPosition + STAGE1_MOTOR_STATE.CLIMBED.motorPosition);
            climberStg1Motor.setPositionPIDFCoefficients(10.0);
            climberStg1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberStg1Motor.setPower(power);
        } else {
            climberStg1Motor.setPower(0);
        }
    }

    public void modifyClimberStg1LengthContinuous(double power){
        climberStg1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg1BrakeModeOn();
        climberStg1Motor.setPower(power);
    }

    //Resets the stage1 climber
    public void resetClimberStg1MotorMode(){
        DcMotorEx.RunMode runMode = climberStg1Motor.getMode();
        climberStg1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climberStg1Motor.setMode(runMode);
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Climber Stage1");
        telemetry.addData("    State", climberStg1MotorState);
        telemetry.addData("    Motor Position", climberStg1Motor.getCurrentPosition());
        telemetry.addLine("=============");
    }

}
