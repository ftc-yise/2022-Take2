package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;


@Autonomous(name = "Auto Red Right", group = "Linear Opmode")
public class AutonomousRedRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // define a series of trajectories
        /*
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 20), Math.toRadians(-90) )
                .build();
        */

        TrajectorySequence traj_seq = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .strafeRight(20)
                .build();

        waitForStart();
        if(isStopRequested()) return;

        // run my trajectories in order
        // drive.followTrajectory(traj1);
        drive.followTrajectorySequence(traj_seq);
        // drive.turn(Math.toRadians(90));
    }
}