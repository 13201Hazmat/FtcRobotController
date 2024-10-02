package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public Servo intakeGripServo;

    public enum INTAKE_GRIP_STATE { //state of the Hand Grip
        OPEN(0.69),//0.65
        OPEN_AUTO(0.65),
        CLOSED(1.0);

        private final double gripPosition;
        INTAKE_GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public INTAKE_GRIP_STATE intakeGripState = INTAKE_GRIP_STATE.CLOSED;

    public enum INTAKE_ARM_STATE{
        DEFAULT(0),
        TRANSFER(-180),
        RAISED(-90),
        PICKUP(90);

        private double armPos;
        //public final int index;
        INTAKE_ARM_STATE(double armPos, ){
            this.armPos = armPos;
            //this.index = index;
        }
    }
    public INTAKE_ARM_STATE intakeArmState = INTAKE_ARM_STATE.TRANSFER;

}
