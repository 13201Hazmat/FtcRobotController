package org.firstinspires.ftc.teamcode.TestOpModes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
@TeleOp(name = "Opencv", group = "Concept")
public class ConceptVisionColorLocatorTest extends LinearOpMode
{
    public RotatedRect boxFit;
    public List<ColorBlobLocatorProcessor.Blob> blobs;
    public double angle = 0.0;
    public double yExtensionFactor = 0.0;
    public int blockX = 0, blockY = 0;
    public int Y_AREA_OF_INTEREST_MAX = 80; // Full Extenstion 1.0
    public double EXTENSION_FACTOR_MAX = 1.0;
    public int Y_AREA_OF_INTEREST_MIN = 180; // Half Extension 0.5
    public double EXTENSION_FACTOR_MIN = 0.5;

    @Override
    public void runOpMode()
    {
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                //.setRoi(ImageRegion.asImageCoordinates(130, 0,  190, 180))
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(2)
                .setErodeSize(7)
                .setDilateSize(4)
                .build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            blobs = colorLocator.getBlobs();

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */
            ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobs);  // filter out very small blobs.
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

            telemetry.addLine(" Area Density Aspect  Center");

            /*
            if (!blobs.isEmpty()) {
                ColorBlobLocatorProcessor.Blob closestBlob = blobs.get(0);
                boxFit = closestBlob.getBoxFit();
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

                blockX = (int) boxFit.center.x;
                blockY = (int) boxFit.center.y;

                yExtensionFactor = EXTENSION_FACTOR_MIN +
                        ((double) (blockY - Y_AREA_OF_INTEREST_MIN) * (EXTENSION_FACTOR_MAX - EXTENSION_FACTOR_MIN)
                                / (double) (Y_AREA_OF_INTEREST_MAX - Y_AREA_OF_INTEREST_MIN));
            } else {
                telemetry.addLine("No blobs detected!");
            }

             */

            if (!blobs.isEmpty()) {
                for (int i = 0; i < blobs.size(); i++) {
                    ColorBlobLocatorProcessor.Blob blob = blobs.get(i);
                    boxFit = blob.getBoxFit();
                    double rawAngle = boxFit.angle;
                    double width = boxFit.size.width;
                    double height = boxFit.size.height;

                    double blobAngle;
                    if (width > height) {
                        blobAngle = Math.min(rawAngle, 90.0 - rawAngle);
                    } else {
                        blobAngle = Math.max(rawAngle, 90.0 - rawAngle);
                    }

                    telemetry.addData("Blob " + i + " Angle", blobAngle);
                }
            } else {
                telemetry.addLine("No blobs detected!");
            }

            telemetry.update();
            sleep(50);
        }
    }
}