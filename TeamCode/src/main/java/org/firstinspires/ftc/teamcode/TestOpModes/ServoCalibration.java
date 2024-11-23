package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoCalibTestTeleOp", group = " 01-Testing")
public class ServoCalibration extends LinearOpMode{
    Servo masterServo;
    String masterServoName;
    String masterServoCalibPosition;
    //double servoSetPosition;
    double servoCurrentPosition;

    /*
    Servos:

    EH Servo 2:        intakeArmServo = hardwareMap.get(Servo.class, "intake_arm");
    RC Servo 2:        intakeWristServo = hardwareMap.get(Servo.class, "intake_wrist");
    RC Servo 0:        intakeSlideServoLeft = hardwareMap.get(Servo.class, "intake_slide_left");
    EH Servo 0:        intakeSlideServoRight = hardwareMap.get(Servo.class, "intake_slide_right");
    RC Servo 3:        gripServo = hardwareMap.get(Servo.class, "specimen_grip");
    RC Servo 4:        outtakeArmServo = hardwareMap.get(Servo.class, "outtake_arm");
    RC Servo 1:        outtakeArmWrist = hardwareMap.get(Servo.class, "outtake_wrist");

    CR Servos:
    EH Servo 4:        intakeGrip = hardwareMap.get(Servo.class, "intake_grip");
    RC Servo 5:        climberStg1ServoLeft = hardwareMap.get(CRServo.class, "climber1_servo_left");
    EH Servo 5:        climberStg1ServoRight = hardwareMap.get(CRServo.class, "climber1_servo_right");
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
            telemetry.addData("    specimen_grip ", "(Dpad Up)");
            telemetry.addData("    outtake_arm   ", "(Dpad left)");
            telemetry.addData("    outtake_wrist ", "(Dpad right)");
            telemetry.addData("    intake_grip", "(Dpad down As Grip)");

            if(gamepad1.x){
                masterServoName = "intake_arm";
                masterServoCalibPosition = "Zero is Intake Arm Vertically downward";
                break;
            }
            if(gamepad1.y){
                masterServoName = "intake_wrist";
                masterServoCalibPosition ="Zero is Horizontallu Facing inward, with Intake Arm in Vertically upward position";
                break;
            }
            if(gamepad1.a){
                masterServoName = "intake_slide_right";
                masterServoCalibPosition = "Max_Extended Right at 0.0";
                break;
            }
            if(gamepad1.b){
                masterServoName = "intake_slide_left";
                masterServoCalibPosition = "Max_Extended Left at 1.0";
                break;
            }
            if(gamepad1.right_bumper){
                masterServoName = "intake_swivel";
                masterServoCalibPosition = "Facing Center is 0.5";
                break;
            }
            if(gamepad1.dpad_up){
                masterServoName = "specimen_grip";
                masterServoCalibPosition = "Full closed is 0.26";
                break;
            }
            if(gamepad1.dpad_left){
                masterServoName = "outtake_arm";
                masterServoCalibPosition = "Fully in mechanical limit inwards is Zero";
                break;
            }
            if(gamepad1.dpad_right){
                masterServoCalibPosition = "Bucket holder parallel to arm is 0.75";
                masterServoName = "outtake_wrist";
                break;
            }

            if(gamepad1.dpad_down){
                masterServoCalibPosition = "Grip Closed is 0.66";
                masterServoName = "intake_grip";
                break;
            }

            telemetry.update();
        }
        masterServo = hardwareMap.get(Servo.class, masterServoName);
    }

    public void runOpMode() throws InterruptedException {
        selectServo();
        telemetry.addData("Selected Servo ", masterServoName);
        telemetry.addData("Calib position ", masterServoCalibPosition);
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
