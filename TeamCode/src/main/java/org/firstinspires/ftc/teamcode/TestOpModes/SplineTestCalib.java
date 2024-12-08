package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;

@TeleOp(name = "Hazmat Spline Calib", group = " 02-Testing")
public final class SplineTestCalib extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .setTangent(Math.toRadians(180))
                        .splineTo(new Vector2d(-15, -15), Math.toRadians(90))
                        //.splineTo(new Vector2d(0, 30), Math.PI)
                        .setReversed(true)
                        .splineTo(new Vector2d(-4, -42), Math.toRadians(135))
                        .build());

    }
}
