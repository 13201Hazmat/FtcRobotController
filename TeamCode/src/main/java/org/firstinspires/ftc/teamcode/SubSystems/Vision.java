package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
import java.util.HashMap;
import java.util.Map;

public class Vision {
    private Telemetry telemetry;
    private ColorBlobLocatorProcessor colorLocator;
    private CameraName camera;
    private VisionPortal portal;

    private static final int X_RANGE = 320;
    private static final int Y_RANGE = 240;
    private static final int Y_AREA_OF_INTEREST_MAX = 30;  // Full extension 1.0
    private static final int Y_AREA_OF_INTEREST_MIN = 210; // Half extension 0.5
    private static final double EXTENSION_FACTOR_MAX = 0.8;
    private static final double EXTENSION_FACTOR_MIN = 0.2;//0.5

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
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.3, 0.95, 0.85, -0.95))
                //.setRoi(ImageRegion.asImageCoordinates(130, 0,  190, 180))
                //.setRoi(ImageRegion.entireFrame())
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

    public void locateFarthestSampleFromRobot() {
        blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(2000, 10000, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        targetBlobDetected = !blobs.isEmpty();

        if (targetBlobDetected) {
            numberOfBlobsDetected = blobs.size();
            ColorBlobLocatorProcessor.Blob farthestBlob = blobs.get(numberOfBlobsDetected-1);
            boxFit = farthestBlob.getBoxFit();

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

            yExtensionFactor = calculateYExtensionFactorFromLookUp(blockY);
        }
        if (yExtensionFactor < 0.1) {
            yExtensionFactor = 0.714;
        }

    }

    public double calculateYExtensionFactorFromLookUp(double yToLookup){
        double answer = 0;
        /*double[][] data = {
                {0.309, 200}, //7 0.315 0.332
                {0.337, 177}, //8 0.343 0.366
                {0.369, 158}, //9 0.384 0.404
                {0.414, 133}, //10 0.424
                {0.450, 115}, //11 0.438
                {0.495, 96}, //12 0.478 0.491
                {0.527, 80}, //13 0.521
                {0.594, 67}, //14 0.562 0.614
                {0.659, 53}, //15 0.685 0.716
                {0.734, 40} , //16 0.740 0.762
                {0.789, 28}  //17
        };*/

        double[][] data = {
                {0.258, 201},//6
                {0.276, 170}, //7
                {0.301, 158}, //8
                {0.338, 129}, //9
                {0.400, 112}, //10
                {0.441, 95}, //11
                {0.495, 77}, //12
                {0.529, 64}, //13
                {0.605, 50}, //14
                {0.663, 39}, //15
                {0.714, 28} , //16
                {0.750, 22}  //17
        };



        // Create the lookup table (using a HashMap for efficient lookups)
        Map<Double, Double> lookupTable = createLookupTable(data);

        if (lookupTable.containsKey(yToLookup)) {
            double xValue = lookupTable.get(yToLookup);
            answer = xValue;
        } else {
            // Handle cases where the y value is not found.  Linear interpolation is one option:
            Double lowerY = null;
            Double upperY = null;
            for (Double y : lookupTable.keySet()) {
                if (y < yToLookup && (lowerY == null || y > lowerY)) {
                    lowerY = y;
                }
                if (y > yToLookup && (upperY == null || y < upperY)) {
                    upperY = y;
                }
            }

            if (lowerY != null && upperY != null) {
                double lowerX = lookupTable.get(lowerY);
                double upperX = lookupTable.get(upperY);

                double interpolatedX = lowerX + (upperX - lowerX) * (yToLookup - lowerY) / (upperY - lowerY);
                answer = interpolatedX;
            }
        }


        return answer;
    }

    // Creates the lookup table from the given data
    static Map<Double, Double> createLookupTable(double[][] data) {
        Map<Double, Double> lookup = new HashMap<>();
        for (double[] row : data) {
            double x = row[0];
            double y = row[1];
            lookup.put(y, x); // y is the key, x is the value
        }
        return lookup;
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