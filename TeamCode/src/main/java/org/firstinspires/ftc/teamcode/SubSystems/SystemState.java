package org.firstinspires.ftc.teamcode.SubSystems;

/**
 * Static Class to capture States of all subsystems and pass to each other
 */

public class SystemState {
    public static Arm.ARM_MOTOR_STATE ArmState = Arm.ARM_MOTOR_STATE.PICKUP;
    public static Hand.GRIP_STATE HandGripState = Hand.GRIP_STATE.OPEN;
    public static Hand.WRIST_STATE HandWristState = Hand.WRIST_STATE.WRIST_LEVEL;
    public static Shoulder.SHOULDER_STATE ShoulderState = Shoulder.SHOULDER_STATE.PICKUP;
    public static Turret.TURRET_MOTOR_STATE TurretState = Turret.TURRET_MOTOR_STATE.FACING_FORWARD;
}