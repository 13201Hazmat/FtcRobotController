package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

public class VisionLimeLight {
    private Telemetry telemetry;
    private Limelight3A limelight;

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
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

    public LLResult result;
    public double sampleOrientation;
    public double xPos;
    public double yPos;

    public void locateNearestSampleFromRobot() {
        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                targetBlobDetected = true;
                pythonOutputs = result.getPythonOutput();
                if (pythonOutputs != null && pythonOutputs.length > 0) {
                    sampleOrientation = pythonOutputs[0];
                    xPos = pythonOutputs[1];
                    yPos = pythonOutputs[2];
                }
            }
        }
        // *** Calculate Extension Factor ***
        yExtensionFactor = EXTENSION_FACTOR_MIN +
                ((double) (-Y_AREA_OF_INTEREST_MIN) * (EXTENSION_FACTOR_MAX - EXTENSION_FACTOR_MIN)
                        / (double) (Y_AREA_OF_INTEREST_MAX - Y_AREA_OF_INTEREST_MIN));
        angle = sampleOrientation;

    }


    public void printDebugMessages() {
        telemetry.addLine("LimeLight Vision Debug");
        telemetry.addData("Name", "%s", limelight.getStatus().getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                limelight.getStatus().getTemp(), limelight.getStatus().getCpu(),(int)limelight.getStatus().getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                limelight.getStatus().getPipelineIndex(), limelight.getStatus().getPipelineType());
        if (result.isValid()) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
            telemetry.addData("Sample Angle:", sampleOrientation);
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("Y Extension Factor", yExtensionFactor);
        telemetry.addLine("=================");
        telemetry.update();
    }
}