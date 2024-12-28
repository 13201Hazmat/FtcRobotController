package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoCalibTestTeleOp", group = " 01-Testing")
public class ServoCalibration extends LinearOpMode{
    Servo masterServo;
    String masterServoName;
    String masterServoCalibPosition;
    double servoStartPosition;
    double servoCurrentPosition;


    /*
    Servos:
RC Servo 0:        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
RC Servo 1:        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist");
RC Servo 2:        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
RC Servo 4:        outtakeArmServo = hardwareMap.get(Servo.class, "outtake_arm");
RC Servo 5:        outtakeGripServo = hardwareMap.get(Servo.class, "outtake_grip");
EH Servo 0:        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
EH Servo 2:        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
EH Servo 3:        intakeSwivelServo = hardwareMap.get(Servo.class, "intake_swivel");
EH Servo 4:        intakeGripServo = hardwareMap.get(Servo.class, "intake_grip");

     */

    public void selectServo(){
        telemetry.setAutoClear(true);
        while(!isStopRequested()){
            telemetry.addLine("Select Servo to caliberate from list");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("    intake_arm   ", "(X/Square)");
            telemetry.addData("    intake_wrist ", "(Y/Triangle)");
            telemetry.addData("    intake_slide_left   ", "(A/Circle)");
            telemetry.addData("    intake_slide_right ", "(B/Cross)");
            telemetry.addData("    intake_swivel ", "(Right Bumper)");
            telemetry.addData("    intake_grip", "(Dpad down)");
            telemetry.addData("    outtake_arm   ", "(Dpad left)");
            telemetry.addData("    outtake_wrist ", "(Dpad right)");
            telemetry.addData("    outtake_grip ", "(Dpad Up)");

            if(gamepad1.x){
                masterServoName = "intake_arm";
                masterServoCalibPosition = "Zero is Intake Arm Vertically downward";
                servoStartPosition = 0.5;
                break;
            }
            if(gamepad1.y){
                masterServoName = "intake_wrist";
                masterServoCalibPosition ="Pickup / Vertically facing down is 1.0";
                servoStartPosition = 1.0;
                break;
            }
            if(gamepad1.a){
                masterServoName = "intake_slide_right";
                masterServoCalibPosition = "Max_Extended Right at 0.0";
                servoStartPosition = 0.0;
                break;
            }
            if(gamepad1.b){
                masterServoName = "intake_slide_left";
                masterServoCalibPosition = "Max_Extended Left at 1.0";
                servoStartPosition = 1.0;
                break;
            }
            if(gamepad1.right_bumper){
                masterServoName = "intake_swivel";
                masterServoCalibPosition = "Facing Center is 0.505";
                servoStartPosition = 0.505;
                break;
            }
            if(gamepad1.dpad_down){
                masterServoCalibPosition = "Grip Closed with block is 0.11";
                masterServoName = "intake_grip";
                servoStartPosition = 0.11;
                break;
            }

            if(gamepad1.dpad_up){
                masterServoName = "outtake_grip";
                masterServoCalibPosition = "Closed with a block is 0.22";
                servoStartPosition = 0.22;
                break;
            }
            if(gamepad1.dpad_left){
                masterServoName = "outtake_arm";
                masterServoCalibPosition = "Fully in mechanical limit inwards is 1.0";
                servoStartPosition = 1.0;
                break;
            }
            if(gamepad1.dpad_right){
                masterServoCalibPosition = "Bucket upright when arm is in physical minimum is 0.55";
                masterServoName = "outtake_wrist";
                servoStartPosition = 0.55;
                break;
            }



            telemetry.update();
        }
        masterServo = hardwareMap.get(Servo.class, masterServoName);
    }

    public void runOpMode() throws InterruptedException {
        selectServo();
        masterServo.setPosition(servoCurrentPosition);
        telemetry.addData("Selected Servo ", masterServoName);
        telemetry.addData("Calib position ", masterServoCalibPosition);
        telemetry.addData("ServoStartPosition", servoStartPosition);
        telemetry.addData("Servo Current Position: ", masterServo.getPosition());
        telemetry.addData("Selected Port  ", masterServo.getPortNumber());
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {
            servoCurrentPosition = masterServo.getPosition();
            if(gp1GetDpad_downPress()){
                if(servoCurrentPosition > -1){ //0
                    masterServo.setPosition(servoCurrentPosition - 0.01);
                }  else {
                    masterServo.setPosition(0);
                }
            }
            if(gp1GetDpad_upPress()){
                if(servoCurrentPosition < 1){
                    masterServo.setPosition(servoCurrentPosition + 0.01);
                }  else {
                    masterServo.setPosition(1);
                }
            }
            telemetry.addData("Selected Servo is :", masterServoName);
            telemetry.addData("Zero location is :", masterServoCalibPosition);
            telemetry.addData("Servo Current Position: ", masterServo.getPosition());
            telemetry.update();
        }
    }

    boolean gp1Dpad_upLast;
    public boolean gp1GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp1Dpad_upLast && gamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast = gamepad1.dpad_up;
        return isPressedDpad_up;
    }

    boolean gp1Dpad_downLast;

    public boolean gp1GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp1Dpad_downLast && gamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = gamepad1.dpad_down;
        return isPressedDpad_down;
    }
}
