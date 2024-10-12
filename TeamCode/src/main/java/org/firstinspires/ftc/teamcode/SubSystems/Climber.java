package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {
    public DcMotorEx climberStg1Motor = null;
    public Servo climberStg1Servo;

    public DcMotorEx climberStg2Motor = null;
    public Servo climberStg2Servo;

    public boolean climber1Activated = false;
    public boolean climber2Activated = false;

    public boolean climbingStg1Started = false;
    public boolean climbingStg2Started = false;

    public enum CLIMBERSTAGE1_SERVO_STATE{
        LOCKED(0.62),
        RELEASED(0.27);

        private double climberStg1ServoPosition;

        CLIMBERSTAGE1_SERVO_STATE(double climberServoPosition){
            this.climberStg1ServoPosition = climberServoPosition;
        }
        public double getClimberStg1ServoPosition(){
            return climberStg1ServoPosition;
        }
    }
    public CLIMBERSTAGE1_SERVO_STATE climberStg1ServoState = CLIMBERSTAGE1_SERVO_STATE.LOCKED;

    public enum CLIMBERSTAGE2_SERVO_STATE{
        LOCKED(0.62),
        RELEASED(0.27);

        private double climberStg2ServoPosition;

        CLIMBERSTAGE2_SERVO_STATE(double climberServoPosition){
            this.climberStg2ServoPosition = climberServoPosition;
        }
        public double getClimberStg2ServoPosition(){
            return climberStg2ServoPosition;
        }
    }
    public CLIMBERSTAGE2_SERVO_STATE climberStg2ServoState = CLIMBERSTAGE2_SERVO_STATE.LOCKED;

    public enum CLIMBERSTAGE1_MOTOR_STATE {
        INITIAL(0), //Position
        CLIMBED(4000), //5000, 117 rpm motor TODO Set value, 6000
        MAX(7500); //9000

        public final int motorPosition;
        CLIMBERSTAGE1_MOTOR_STATE(int motorPosition) {
            this.motorPosition = motorPosition;
        }
    }
    public CLIMBERSTAGE1_MOTOR_STATE climberStg1MotorState = CLIMBERSTAGE1_MOTOR_STATE.INITIAL;

    public double climberMotorStg1CurrentPosition = climberStg1MotorState.motorPosition;
    public double climberMotorStg1NewPosition = climberStg1MotorState.motorPosition;

    public enum CLIMBERSTAGE2_MOTOR_STATE {
        INITIAL(0), //Position
        CLIMBED(4000), //5000, 117 rpm motor TODO Set value, 6000
        MAX(7500); //9000

        public final int motorPosition;
        CLIMBERSTAGE2_MOTOR_STATE(int motorPosition) {
            this.motorPosition = motorPosition;
        }
    }
    public CLIMBERSTAGE2_MOTOR_STATE climberStg2MotorState = CLIMBERSTAGE2_MOTOR_STATE.INITIAL;

    public double climberMotorStg2CurrentPosition = climberStg2MotorState.motorPosition;
    public double climberMotorStg2NewPosition = climberStg2MotorState.motorPosition;

    public boolean runClimberMotorToLevelState = false;

    public double startCurrentPosition;

    public Telemetry telemetry;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberStg1Motor = hardwareMap.get(DcMotorEx.class, "climber_stage1_motor");
        climberStg1Servo = hardwareMap.get(Servo.class, "climber_stage1_servo");
        climberStg2Motor = hardwareMap.get(DcMotorEx.class, "climber_stage2_motor");
        climberStg2Servo = hardwareMap.get(Servo.class, "climber_stage2_servo");
        ascendClimberStg1Servo();
        ascendClimberStg2Servo();
        initClimber();
    }

    public void ascendClimberStg1Servo(){
        climberStg1Servo.setPosition(CLIMBERSTAGE1_SERVO_STATE.LOCKED.getClimberStg1ServoPosition());
        climberStg1ServoState = CLIMBERSTAGE1_SERVO_STATE.LOCKED;
    }

    public void ascendClimberStg2Servo(){
        climberStg2Servo.setPosition(CLIMBERSTAGE2_SERVO_STATE.LOCKED.getClimberStg2ServoPosition());
        climberStg2ServoState = CLIMBERSTAGE2_SERVO_STATE.LOCKED;
    }

    public void initClimber(){
        climberStg1Motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberStg1Motor.setPositionPIDFCoefficients(10.0); //5
        climberStg2Motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberStg2Motor.setPositionPIDFCoefficients(10.0); //5
        startCurrentPosition = climberStg1Motor.getCurrentPosition();
        turnClimberStg1BrakeModeOff();
        turnClimberStg2BrakeModeOff();
    }

    public void turnClimberStg1BrakeModeOn(){
        climberStg1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnClimberStg2BrakeModeOn(){
        climberStg2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnClimberStg1BrakeModeOff(){
        climberStg1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void turnClimberStg2BrakeModeOff(){
        climberStg2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void moveClimberStg1Motor(CLIMBERSTAGE1_MOTOR_STATE toClimberMotorState){
        turnClimberStg1BrakeModeOn();
        climberMotorStg1CurrentPosition = climberStg1Motor.getCurrentPosition();
        climberStg1Motor.setTargetPosition((int)(toClimberMotorState.motorPosition + startCurrentPosition));
        climberStg1MotorState = toClimberMotorState;
        runClimberMotorToLevelState = true;
        runClimberStg1MotorToLevel();
    }

    public void moveClimberStg2Motor(CLIMBERSTAGE2_MOTOR_STATE toClimberMotorState){
        turnClimberStg2BrakeModeOn();
        climberMotorStg2CurrentPosition = climberStg2Motor.getCurrentPosition();
        climberStg2Motor.setTargetPosition((int)(toClimberMotorState.motorPosition + startCurrentPosition));
        climberStg2MotorState = toClimberMotorState;
        runClimberMotorToLevelState = true;
        runClimberStg2MotorToLevel();
    }

    //sets the ClimberStage1 motor power
    public void runClimberStg1MotorToLevel(){
        double power = 1.0;

        if (climberStg1MotorState == CLIMBERSTAGE1_MOTOR_STATE.INITIAL) {
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

    //sets the ClimberStage2 motor power
    public void runClimberStg2MotorToLevel(){
        double power = 1.0;

        if (climberStg2MotorState == CLIMBERSTAGE2_MOTOR_STATE.INITIAL) {
            turnClimberStg1BrakeModeOff();
        } else {
            turnClimberStg2BrakeModeOn();
        }

        climberStg2Motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runClimberMotorToLevelState){
            climberStg2Motor.setPower(power);
            /*TODO if (!outtakeMotor.isBusy()) */runClimberMotorToLevelState = false;
        } else{
            climberStg2Motor.setPower(0.0);
        }
    }

    public void moveClimberStg1UpInSteps(double power){
        climberStg1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg1BrakeModeOn();

        int climberMotorCurrentPosition = climberStg1Motor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < CLIMBERSTAGE1_MOTOR_STATE.MAX.motorPosition)){
            climberStg1Motor.setTargetPosition(climberMotorCurrentPosition + CLIMBERSTAGE1_MOTOR_STATE.CLIMBED.motorPosition);
            climberStg1Motor.setPositionPIDFCoefficients(10.0);
            climberStg1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberStg1Motor.setPower(power);
        } else {
            climberStg1Motor.setPower(0);
        }
    }

    public void moveClimberStg2UpInSteps(double power){
        climberStg2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg2BrakeModeOn();

        int climberMotorCurrentPosition = climberStg2Motor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < CLIMBERSTAGE2_MOTOR_STATE.MAX.motorPosition)){
            climberStg2Motor.setTargetPosition(climberMotorCurrentPosition + CLIMBERSTAGE2_MOTOR_STATE.CLIMBED.motorPosition);
            climberStg2Motor.setPositionPIDFCoefficients(10.0);
            climberStg2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberStg2Motor.setPower(power);
        } else {
            climberStg2Motor.setPower(0);
        }
    }

    public void moveClimberStg1DownInSteps(double power){
        climberStg1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg1BrakeModeOn();

        int climberMotorCurrentPosition = climberStg1Motor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition > CLIMBERSTAGE1_MOTOR_STATE.INITIAL.motorPosition)){
            climberStg1Motor.setTargetPosition(climberMotorCurrentPosition - CLIMBERSTAGE1_MOTOR_STATE.CLIMBED.motorPosition);
            climberStg1Motor.setPositionPIDFCoefficients(10.0);
            climberStg1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberStg1Motor.setPower(power);
        } else {
            climberStg1Motor.setPower(0);
        }
    }

    public void moveClimberStg2DownInSteps(double power){
        climberStg2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg2BrakeModeOn();

        int climberMotorCurrentPosition = climberStg2Motor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition > CLIMBERSTAGE2_MOTOR_STATE.INITIAL.motorPosition)){
            climberStg2Motor.setTargetPosition(climberMotorCurrentPosition - CLIMBERSTAGE2_MOTOR_STATE.CLIMBED.motorPosition);
            climberStg2Motor.setPositionPIDFCoefficients(10.0);
            climberStg2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberStg2Motor.setPower(power);
        } else {
            climberStg2Motor.setPower(0);
        }
    }

    public void modifyClimberStg1LengthContinuous(double power){
        climberStg1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg1BrakeModeOn();
        climberStg1Motor.setPower(power);
    }

    public void modifyClimberStg2LengthContinuous(double power){
        climberStg2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberStg2BrakeModeOn();
        climberStg2Motor.setPower(power);
    }

    //Resets the stage1 climber
    public void resetClimberStg1MotorMode(){
        DcMotorEx.RunMode runMode = climberStg1Motor.getMode();
        climberStg1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climberStg1Motor.setMode(runMode);
    }

    //Resets the stage2 climber
    public void resetClimberStg2MotorMode(){
        DcMotorEx.RunMode runMode = climberStg2Motor.getMode();
        climberStg2Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climberStg2Motor.setMode(runMode);
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Climber Stage1");
        telemetry.addData("    State", climberStg1MotorState);
        telemetry.addData("    Motor Position", climberStg1Motor.getCurrentPosition());
        telemetry.addLine("=============");
        telemetry.addLine("Climber Stage2");
        telemetry.addData("    State", climberStg2MotorState);
        telemetry.addData("    Motor Position", climberStg2Motor.getCurrentPosition());
    }

}
