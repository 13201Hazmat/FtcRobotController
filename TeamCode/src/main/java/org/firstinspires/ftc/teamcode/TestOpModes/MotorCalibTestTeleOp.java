package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GameOpModes.FTCWiresAutoIntoTheDeep;

@TeleOp(name = "MotorCalibTestTeleOp", group = " 01 Testing")
public class MotorCalibTestTeleOp extends LinearOpMode{
    DcMotorEx masterMotor;
    String masterMotorName;
    int motorCurrentPosition;
    /*
    RC Motor 1:        specimenSlide =
    RC Motor 2:        outtakeSlideLeft = hardwareMap.get(DcMotorEx.class, "outtake_slides_left");
    EH Motor 2:        outtakeSlideRight = hardwareMap.get(DcMotorEx.class, "outtake_slides_right");
    EH Motor 1:        climberStg1Motor = hardwareMap.get(DcMotorEx.class, "climber1_motor");
     */

    public void selectMotor(){
        while(!isStopRequested()){
            telemetry.addLine("Select Motor to caliberate from list");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("    specimen_slide   ", "(X/Square)");
            telemetry.addData("    outtake_slides_left ", "(Y/Triangle)");
            telemetry.addData("    outtake_slides_right   ", "(A/Circle)");
            telemetry.addData("    climber1_motor ", "(B/Cross)");

            if(gamepad1.x){
                masterMotorName = "specimen_slide";
                break;
            }
            if(gamepad1.y){
                masterMotorName = "outtake_slides_left";
                break;
            }
            if(gamepad1.b){
                masterMotorName = "outtake_slides_right";
                break;
            }
            if(gamepad1.a){
                masterMotorName = "climber1_motor";
                break;
            }
            telemetry.update();
        }
        masterMotor = hardwareMap.get(DcMotorEx.class, masterMotorName);
    }

    public void runOpMode() throws InterruptedException {
        selectMotor();
        telemetry.addData("Selected Motor", masterMotorName);
        telemetry.addData("   Port ", masterMotor.getPortNumber());
        telemetry.addData("Direction ", masterMotor.getDirection());

        telemetry.update();
        masterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        masterMotor.setPositionPIDFCoefficients(10.0); //5
        masterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
           motorCurrentPosition = masterMotor.getCurrentPosition();
            while(gp1GetDpad_up() && !isStopRequested()){
                masterMotor.setTargetPosition(motorCurrentPosition + 100);
                masterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                masterMotor.setPower(0.5);
            }
            while(gp1GetDpad_down() && !isStopRequested()){
                masterMotor.setTargetPosition(motorCurrentPosition - 100);
                masterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                masterMotor.setPower(0.5);
            }
            masterMotor.setPower(0);
            telemetry.addData("Motor Current Position: ", masterMotor.getCurrentPosition());
            telemetry.update();
        }
    }



    public boolean gp1GetDpad_up(){
        return gamepad1.dpad_up;
    }
    public boolean gp1GetDpad_down(){
        return gamepad1.dpad_down;
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
