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
    public int Y_RANGE = 240;
    public int Y_AREA_OF_INTEREST_MAX = 125; // Full Extenstion 1.0
    public double EXTENSION_FACTOR_MAX = 1.0;
    public int Y_AREA_OF_INTEREST_MIN = 25; // Half Extension 0.5
    public double EXTENSION_FACTOR_MIN = 0.5;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        CameraName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.5, 1, 0.75, -0.25))
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in imag/e
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
    public double angle = 0.0;

    public ColorRange targetColor;
    public List<ColorBlobLocatorProcessor.Blob> blobs;
    public RotatedRect boxFit;
    public int blockX = 0, blockY = 0;
    public int numberOfBlobsDetected = 0;

    public void locateNearestSamplefromRobot() {
        final double pixelToExtensionScale = 0.05;

        blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        targetBlobDetected = !blobs.isEmpty();

        if (targetBlobDetected) {
            numberOfBlobsDetected = blobs.size();
            ColorBlobLocatorProcessor.Blob closestBlob = blobs.get(0);
            boxFit = closestBlob.getBoxFit();

            blockX = (int) boxFit.center.x;
            blockY = (int) boxFit.center.y;

            //xExtensionFactor = 1.0 - (double) blockX / (double) X_RANGE;
            yExtensionFactor = EXTENSION_FACTOR_MIN +
                    ((double) (blockY - Y_AREA_OF_INTEREST_MIN) * (EXTENSION_FACTOR_MAX - EXTENSION_FACTOR_MIN)
                    / (double) (Y_AREA_OF_INTEREST_MAX - Y_AREA_OF_INTEREST_MIN));
            angle =  boxFit.angle;
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
        //telemetry.addData("    xExtensionFactor", xExtensionFactor);
        telemetry.addData("    yExtensionFactor", yExtensionFactor);
        telemetry.addLine("=============");
    }

}
