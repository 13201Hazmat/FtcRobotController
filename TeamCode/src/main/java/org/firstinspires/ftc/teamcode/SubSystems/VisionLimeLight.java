package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.RotatedRect;

public class VisionLimeLight {
    private Telemetry telemetry;
    private Limelight3A limelight;
    public Servo limeLightArm;

    public enum ARM_STATE{
        RETRACTED(0.1),
        EXTENDED(0.7);

        private final double armPos;
        ARM_STATE(double armPos) {this.armPos = armPos;}
    }

    public ARM_STATE limelightArmState = ARM_STATE.RETRACTED;
    public double ARM_DELTA = 0.05;

    public void moveArmForward(){
        limeLightArm.setPosition(limeLightArm.getPosition() + ARM_DELTA);
        telemetry.addData("Arm Pos", limeLightArm.getPosition());
        telemetry.update();
    }

    public void moveArmBackward(){
        limeLightArm.setPosition(limeLightArm.getPosition() - ARM_DELTA);
        telemetry.addData("Arm Pos", limeLightArm.getPosition());
        telemetry.update();
    }

    public void extendArm(){
        limeLightArm.setPosition(ARM_STATE.EXTENDED.armPos);
    }

    public void retractArm(){
        limeLightArm.setPosition(ARM_STATE.RETRACTED.armPos);
    }

    private static final int X_RANGE = 320;
    private static final int Y_RANGE = 240;
    private static final int Y_AREA_OF_INTEREST_MAX = 80;  // Full extension 1.0
    private static final int Y_AREA_OF_INTEREST_MIN = 180; // Half extension 0.5
    private static final double EXTENSION_FACTOR_MAX = 1.0;
    private static final double EXTENSION_FACTOR_MIN = 0.5;

    public boolean targetBlobDetected = false;
    public double yExtensionFactor = 0.0;
    public double angle = 0.0;

    public double[] pythonOutputs;
    private RotatedRect boxFit;
    private int blockX = 0, blockY = 0;
    private int numberOfBlobsDetected = 0;

    public VisionLimeLight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limeLightArm = hardwareMap.get(Servo.class, "limelight_arm");
        retractArm();
    }

    public void startLimelight() {
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void stopLimeLight() {
        limelight.stop();
    }

    public double[] result;
    public double xPos;
    public double yPos;

    public void locateNearestSampleFromRobot() {
        result = limelight.getLatestResult().getPythonOutput();
        angle = result[0];
        xPos = result[1];
        yPos = result[2];
        /*
        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                targetBlobDetected = true;
                pythonOutputs = result.getPythonOutput();
                if (pythonOutputs != null && pythonOutputs.length > 0) {
                    angle = pythonOutputs[0];
                    xPos = pythonOutputs[1];
                    yPos = pythonOutputs[2];
                }
            }
        }
        // *** Calculate Extension Factor ***
        yExtensionFactor = EXTENSION_FACTOR_MIN +
                ((double) (-Y_AREA_OF_INTEREST_MIN) * (EXTENSION_FACTOR_MAX - EXTENSION_FACTOR_MIN)
                        / (double) (Y_AREA_OF_INTEREST_MAX - Y_AREA_OF_INTEREST_MIN));

         */
    }


    public void printDebugMessages() {
        telemetry.addLine("LimeLight Vision Debug");
        telemetry.addData("Name", "%s", limelight.getStatus().getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                limelight.getStatus().getTemp(), limelight.getStatus().getCpu(),(int)limelight.getStatus().getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                limelight.getStatus().getPipelineIndex(), limelight.getStatus().getPipelineType());
        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.addData("Angle", angle);
        telemetry.addData("Y Extension Factor", yExtensionFactor);
        telemetry.addLine("=================");
        telemetry.update();
    }
}