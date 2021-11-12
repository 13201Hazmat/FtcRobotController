package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Static Class to define Gamefield Vector positions.
 * These are used in start Position estimates and in automatic targetic.
 *
 * The static class also has PosStorage defined, to pass the last position in autonomous mode
 * to following TeleOp mode
 */
public class GameField {
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Pose2d ORIGINPOSE = new Pose2d(0,0,Math.toRadians(0));

    // Declare and assign starting pose of robot
    //TODO: Update start position correctly.
    public static final Pose2d BLUE_WAREHOUSE_STARTPOS =  new Pose2d(-63,12,Math.toRadians(180));
    public static final Pose2d BLUE_CAROUSAL_STARTPOS =  new Pose2d(-63,-36,Math.toRadians(180));
    public static final Pose2d RED_WAREHOUSE_STARTPOS =  new Pose2d(63,12,Math.toRadians(0)); //y=-26
    public static final Pose2d RED_CAROUSAL_STARTPOS =  new Pose2d(63,-36,Math.toRadians(0));

    //Declare locations of key positions on field
    //Example - public static final Vector2d BLUE_TOWER_GOAL = new Vector2d(72,42);
    public static final Vector2d RED_CAPSTONE_WAREHOUSE_LEVEL1POS = new Vector2d(49,-2);
    public static final Vector2d RED_CAPSTONE_WAREHOUSE_LEVEL2POS = new Vector2d(49,7);
    public static final Vector2d RED_CAPSTONE_WAREHOUSE_LEVEL3POS = new Vector2d(49,16);

    public static final Vector2d BLUE_CAPSTONE1POS = new Vector2d(-51,9);
    public static final Vector2d BLUE_CAPSTONE2POS = new Vector2d(-51,18);
    public static final Vector2d BLUE_CAPSTONE3POS = new Vector2d(-51,27);

    public static final Vector2d RED_CAPSTONE_STORAGE_LEVEL1POS = new Vector2d(49,-20);
    public static final Vector2d RED_CAPSTONE_STORAGE_LEVEL2POS = new Vector2d(49,-29);
    public static final Vector2d RED_CAPSTONE_STORAGE_LEVEL3POS = new Vector2d(49,-38);


    //Define and declare Playing Alliance
    public enum PLAYING_ALLIANCE{
        RED_ALLIANCE,
        BLUE_ALLIANCE,
    }
    public static PLAYING_ALLIANCE playingAlliance = PLAYING_ALLIANCE.BLUE_ALLIANCE;
    public static double ALLIANCE_FACTOR = 1;


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        STARTPOS_1,
        STARTPOS_2
        //TODO: Update names for Startpos
    }
    public static START_POSITION startPosition = START_POSITION.STARTPOS_1;

    //Define targets for Vision to determine Autonomous mode action based on camera detection
    public enum VISION_IDENTIFIED_TARGET {
        LEVEL1,
        LEVEL2,
        LEVEL3,
        UNKNOWN;
    };

    public enum VISION_IDENTIFIER{
        DUCK,
        MARKER
    };
    public static VISION_IDENTIFIER visionIdentifier = VISION_IDENTIFIER.DUCK;

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d();

}
