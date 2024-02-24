package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.dnn.DetectionModel;

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

public class Lights {

    public Telemetry telemetry;
    public RevBlinkinLedDriver blinkinLedDriver;

    public enum REV_BLINKIN_PATTERN {
        DEMO(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW),
        NONE(RevBlinkinLedDriver.BlinkinPattern.BLACK),
        BACK_DROP_RED(RevBlinkinLedDriver.BlinkinPattern.RED),
        BACK_DROP_AMBER(RevBlinkinLedDriver.BlinkinPattern.GREEN),
        ONE_IN_MAGAZINE(RevBlinkinLedDriver.BlinkinPattern.YELLOW),
        TWO_IN_MAGAZINE(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN),
        END_GAME(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE),
        DEFAULT(RevBlinkinLedDriver.BlinkinPattern.GOLD);

        private RevBlinkinLedDriver.BlinkinPattern blinkinPattern;
        private REV_BLINKIN_PATTERN(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
            this.blinkinPattern = blinkinPattern;
        };
    }
    public REV_BLINKIN_PATTERN currentRevBlinkinPattern;

    public Lights(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        setPattern(REV_BLINKIN_PATTERN.DEMO);
    }

    public void setPattern(REV_BLINKIN_PATTERN revBlinkinPattern) {
        if (revBlinkinPattern != currentRevBlinkinPattern) {
            blinkinLedDriver.setPattern(revBlinkinPattern.blinkinPattern);
            currentRevBlinkinPattern = revBlinkinPattern;
        }
    }

    public void printDebugMessages(){
        telemetry.addData("Current Blinking Pattern:", currentRevBlinkinPattern);
        telemetry.addLine("=================");
    }
}
