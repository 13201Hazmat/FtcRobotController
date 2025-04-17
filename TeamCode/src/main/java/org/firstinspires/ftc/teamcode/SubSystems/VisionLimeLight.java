package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.Vision.createLookupTable;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.RotatedRect;

import java.util.HashMap;
import java.util.Map;

public class VisionLimeLight {
    private Telemetry telemetry;
    private Limelight3A limelight;
    public Servo limeLightArm;

    public enum ARM_STATE{
        RETRACTED(1),
        EXTENDED(0.72),
        CLIMB(0.54);

        private final double armPos;
        ARM_STATE(double armPos) {this.armPos = armPos;}
    }

    public ARM_STATE limelightArmState = ARM_STATE.RETRACTED;
    public double ARM_DELTA = 0.01;

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

/*    public void extendArm(){
        limeLightArm.setPosition(ARM_STATE.EXTENDED.armPos);
        limelightArmState = ARM_STATE.EXTENDED;
    }

    public void retractArm(){
        limeLightArm.setPosition(ARM_STATE.RETRACTED.armPos);
        limelightArmState = ARM_STATE.RETRACTED;
    }

    public void moveArmClimb(){
        limeLightArm.setPosition(ARM_STATE.CLIMB.armPos);
        limelightArmState = ARM_STATE.CLIMB;
    }
*/
    private static final int X_RANGE = 320;
    private static final int Y_RANGE = 240;
    private static final int Y_AREA_OF_INTEREST_MAX = 80;  // Full extension 1.0
    private static final int Y_AREA_OF_INTEREST_MIN = 180; // Half extension 0.5
    private static final double EXTENSION_FACTOR_MAX = 1.0;
    private static final double EXTENSION_FACTOR_MIN = 0.5;
    private static final double INTAKE_XPOS = 550;
    private static final double INCHES_PER_PIXEL = 9.25/520; //9.25 inches for 520 pixeld
    public double inchesToStrafe;

    public boolean targetBlobDetected = false;
    public double yExtensionFactor = 0.0;
    public double angle = 0.0;

    public VisionLimeLight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
    }

    public void startLimelight() {
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void stopLimeLight() {
        limelight.stop();
    }

    public double[] result = {};
    public double xPos;
    public double yPos;

    public void locateNearestSampleFromRobot() {
        if (limelight.isRunning()) {
            result = limelight.getLatestResult().getPythonOutput();
            if (result == null || result.length < 3) {
                targetBlobDetected = false;
            } else {
                angle = result[0];
                xPos = result[1];
                yPos = result[2];

                if (xPos > 51 && xPos < 605 && yPos > 41 && yPos < 403) {
                    targetBlobDetected = true;

                    //inchesToStrafe = (xPos - INTAKE_XPOS) * INCHES_PER_PIXEL*2.73;
                    inchesToStrafe = calculateInchesToStrafeFromLookUp(xPos);
                    yExtensionFactor = calculateYExtensionFactorFromLookUp(yPos);

                    if (yExtensionFactor < 0.1) {
                        yExtensionFactor = 0.514;
                    }
                } else {
                    targetBlobDetected = false;
                }
            }
        } else {
            targetBlobDetected = false;
        }
    }

    public double calculateYExtensionFactorFromLookUp(double yPos) {
        double xtensionFactor = 0;
        xtensionFactor = (-0.0006664865 * yPos) + 0.4476567575;
        return xtensionFactor;
    }

    public double calculateInchesToStrafeFromLookUp(double xPos){
        double answer = 0;

        double[][] data = {
                {3.50 , 598}, //0 Good
                {0.29 , 564}, //1 Good
                {-4.40 , 507}, //2 Good
                {-5.70 , 455}, //3 Good
                {-8.25 , 390}, //4 Good
                {-9.50 , 337}, //5 Good
                {-11.25 , 285}, //6 Good
                {-12.95 , 228}, //7 Good
                {-13.75 , 164}, //8 good
                {-14.50 , 112}, //9 Good
                {-16.00 , 57}, //10 Good
                {-17.50 , 18}
        };



        // Create the lookup table (using a HashMap for efficient lookups)
        Map<Double, Double> lookupTable = createLookupTable(data);

        if (lookupTable.containsKey(xPos)) {
            double xValue = lookupTable.get(xPos);
            answer = xValue;
        } else {
            // Handle cases where the y value is not found.  Linear interpolation is one option:
            Double lowerY = null;
            Double upperY = null;
            for (Double y : lookupTable.keySet()) {
                if (y < xPos && (lowerY == null || y > lowerY)) {
                    lowerY = y;
                }
                if (y > xPos && (upperY == null || y < upperY)) {
                    upperY = y;
                }
            }

            if (lowerY != null && upperY != null) {
                double lowerX = lookupTable.get(lowerY);
                double upperX = lookupTable.get(upperY);

                double interpolatedX = lowerX + (upperX - lowerX) * (xPos - lowerY) / (upperY - lowerY);
                answer = interpolatedX;
            }
        }
        return answer;
    }

    public void printDebugMessages() {
        telemetry.addLine("LimeLight Vision Debug");
        telemetry.addData("Name", "%s", limelight.getStatus().getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                limelight.getStatus().getTemp(), limelight.getStatus().getCpu(),(int)limelight.getStatus().getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                limelight.getStatus().getPipelineIndex(), limelight.getStatus().getPipelineType());
        telemetry.addData("Position", "xPos %.2f, yPos %.2f, Angle %.2f", xPos, yPos, angle);
        //telemetry.addData("yPos", yPos);
        //telemetry.addData("Angle", angle);
        telemetry.addData("Y Extension Factor", yExtensionFactor);
        telemetry.addData("inchesToStrafe",inchesToStrafe);
        telemetry.addLine("=================");
        //telemetry.update();
    }
}