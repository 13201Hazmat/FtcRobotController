package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.AadiPose;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * FTC WIRES Autonomous Example
 */
@Autonomous(name = "FTC Wires Autonomous", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class AutoOpModeAadi extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public Vision vision;
    public DriveTrain driveTrain;
    public Arm arm;
    public Shoulder shoulder;
    public Turret turret;
    public GamepadController gamepadController;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        vision = new Vision(hardwareMap);
        arm = new Arm(hardwareMap);
        shoulder = new Shoulder(hardwareMap);
        turret = new Turret(hardwareMap);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        //Build Autonomous trajectory to be used based on starting position selected
        buildAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addData("Start FTC Wires (ftcwires.org) Autonomous Mode adopted for Team","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", vision.visionIdentifiedTarget);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Stop Vision process
            vision.deactivateVuforiaTensorFlow();

            //Build parking trajectory based on last detected target by vision
            buildParking();

            //run Autonomous trajectory
            runAutoAndParking();
        }

        //Trajectory is completed, display Parking complete
        parkingComplete();
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking ;

    //Initialize any other Pose2d's as desired
    AadiPose aadi;

    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d parkPose;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, 11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, -11, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, -11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, 11, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, 36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, 60, Math.toRadians(0)); break; // Location 3
                }
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(midWayPose)
                //turn turret and pick, then drop cone
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        trajectoryParking = driveTrain.trajectorySequenceBuilder(midWayPose)
                .lineToLinearHeading(parkPose)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(){
        telemetry.setAutoClear(false);
        telemetry.addData("Running FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryAuto);
        driveTrain.followTrajectorySequence(trajectoryParking);
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void parkingComplete(){
        telemetry.addData("Parked in Location", vision.visionIdentifiedTarget);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepadController.gp1GetButtonXPress()){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepadController.gp1GetButtonAPress()){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}
