package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Size;

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

public class Vision {
    private Telemetry telemetry;
    private ColorBlobLocatorProcessor colorLocator;
    private CameraName camera;
    private VisionPortal portal;
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

    public List<ColorBlobLocatorProcessor.Blob> blobs;
    private RotatedRect boxFit;
    private int blockX = 0, blockY = 0;
    private int numberOfBlobsDetected = 0;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                //.setRoi(ImageRegion.asImageCoordinates(130, 0,  190, 180))
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(2)
                .setErodeSize(7)
                .setDilateSize(4)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(X_RANGE, Y_RANGE))
                .setCamera(camera)
                .build();
    }

    public void locateNearestSampleFromRobot() {
        blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        targetBlobDetected = !blobs.isEmpty();

        if (targetBlobDetected) {
            numberOfBlobsDetected = blobs.size();
            ColorBlobLocatorProcessor.Blob closestBlob = blobs.get(0);
            boxFit = closestBlob.getBoxFit();

            blockX = (int) boxFit.center.x;
            blockY = (int) boxFit.center.y;

            // *** Calculate Angle ***
            double rawAngle = boxFit.angle;
            double width = boxFit.size.width;
            double height = boxFit.size.height;

            if (width > height) {
                // Near horizontal case
                angle = Math.min(rawAngle, 90.0 - rawAngle);
            } else {
                // Near vertical case
                angle = Math.max(rawAngle, 90.0 - rawAngle);
            }

            // *** Calculate Extension Factor ***
            yExtensionFactor = EXTENSION_FACTOR_MIN +
                    ((double) (blockY - Y_AREA_OF_INTEREST_MIN) * (EXTENSION_FACTOR_MAX - EXTENSION_FACTOR_MIN)
                            / (double) (Y_AREA_OF_INTEREST_MAX - Y_AREA_OF_INTEREST_MIN));
        }
    }

    public void printDebugMessages() {
        telemetry.addLine("Vision Debug");
        telemetry.addData("ROI Range", "(%d, %d)", X_RANGE, Y_RANGE);
        telemetry.addData("Target Blob Detected", targetBlobDetected);
        telemetry.addData("Number of Blobs", numberOfBlobsDetected);
        telemetry.addData("Block Position", "(%d, %d)", blockX, blockY);
        telemetry.addData("Angle", angle);
        telemetry.addData("Y Extension Factor", yExtensionFactor);
        telemetry.addLine("=================");
        telemetry.update();
    }
}