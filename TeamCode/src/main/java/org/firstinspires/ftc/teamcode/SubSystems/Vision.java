package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class Vision {
    public Telemetry telemetry;
    public ColorBlobLocatorProcessor colorLocator;
    public CameraName camera;
    public int X_RANGE = 320;
    public int Y_RANGE = 640;
    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        CameraName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0., 0.25, -0.25))  // search Left 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(X_RANGE, Y_RANGE))
                .setCamera(camera)
                .build();
    }

    public boolean targetBlobDetected = false;
    public double yExtensionFactor = 0.0;
    public double xExtensionFactor = 0.0;

    public ColorRange targetColor;
    public List<ColorBlobLocatorProcessor.Blob> blobs;
    public RotatedRect boxFit;
    public int blockX = 0, blockY = 0;
    public int numberOfBlobsDetected = 0;

    public void locateNearestSamplefromRobot(ColorRange toTargetColor) {
        targetColor = toTargetColor;
        final double pixelToExtensionScale = 0.05;

        blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        targetBlobDetected = !blobs.isEmpty();

        if (targetBlobDetected) {
            numberOfBlobsDetected = blobs.size();
            ColorBlobLocatorProcessor.Blob closestBlob = blobs.get(numberOfBlobsDetected - 1);
            boxFit = closestBlob.getBoxFit();

            blockX = (int) boxFit.center.x;
            blockY = (int) boxFit.center.y;

            xExtensionFactor = 1.0 - (double) blockX / (double) X_RANGE;
            yExtensionFactor = (double) blockY / (double) Y_RANGE;
        }
    }

    public void printDebugMessages(){
        //******  debug ******
        telemetry.addLine("Vision");
        telemetry.addData("    ROI range", "(%d, %d)", X_RANGE, Y_RANGE);
        telemetry.addData("    Target Color", targetColor);
        telemetry.addData("    Target Blob Detected", targetBlobDetected);
        telemetry.addData("    Number of Blobs", blobs.size() );
        telemetry.addData("    Closest Block Position", "(%d, %d)", blockX, blockY);
        telemetry.addData("    xExtensionFactor", xExtensionFactor);
        telemetry.addData("    yExtensionFactor", yExtensionFactor);
        telemetry.addLine("=============");
    }

}
