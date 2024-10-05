package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeArm {
    public Servo outtakeArmServo;

    public enum OUTTAKE_ARM_STATE{
        INIT(0),
        TRANSFER(0.5),
        DROP(0.8),
        MAX(1.0);

        private double armPos;
        OUTTAKE_ARM_STATE(double armPos){
            this.armPos = armPos;
        }
    }
    public OUTTAKE_ARM_STATE outtakeArmState = OUTTAKE_ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    public OuttakeArm(HardwareMap hardwareMap) { //map hand servo's to each
        outtakeArmServo = hardwareMap.get(Servo.class, "outtake_arm");
        initOuttakeArm();
    }

    public void initOuttakeArm(){
        moveArm(OUTTAKE_ARM_STATE.INIT);
    }

    public void moveArm(OUTTAKE_ARM_STATE outtakeArmState){
        outtakeArmServo.setPosition(outtakeArmState.armPos);
        this.outtakeArmState = outtakeArmState;
    }

}

