package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Auto Red Right", group = "Linear Opmode")
public class AutonomousRedRight extends LinearOpMode {
    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------

        // create instance of roadrunner drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // create instance of yise lift arm class
        liftArm arm = new liftArm(hardwareMap);

        // create variable for grabber
        Servo coneGrabber;
        coneGrabber = hardwareMap.get(Servo.class, "cone_grabber");

        waitForStart();
        if(isStopRequested()) return;

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // example trajectory sequences
        // based on testing, there's no problem doing forward and strafeRight in a row

        // seq_1:
        // Drive forward 20 inches (after first 10 inches, start raising arm to top cone height)
        // Then strafe right 20 inches and turn right 90 degrees
        //
        // This is an example of a "global" displacement marker, it actually doesn't matter where
        // it is placed in the order of the sequence. It will run after the bot has moved
        // a total of 10 inches.
        //
        // If you change the marker from 10 to 30, it will run in the middle of the stafeRight()
        // Note: 1 marker, when written in this syntax, can include multiple actions
        TrajectorySequence seq_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .addDisplacementMarker(10, () -> {
                    arm.getTopCone();
                })
                .strafeRight(20)
                .turn(-90)
                .build();

        // seq_2:
        // Back up 10 inches, after backup is fully complete, lower arm to ground junction height
        // Then turn 180 degrees to the left
        //
        // Note: starting position for 2nd sequence uses the end() method of the 1st sequence
        //
        // This is an example of an "inline" displacement marker. Notice there's no distance given
        // This will run immediate after the back() command completes (ie order DOES matter)
        TrajectorySequence seq_2 = drive.trajectorySequenceBuilder(seq_1.end())
                .back(10)
                .addDisplacementMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.HOVER);
                })
                .turn(180)
                .build();

        // run my trajectories in order

        // drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(seq_1);
        // close grabber
        coneGrabber.setPosition(Servo.MIN_POSITION);
        // back up, turn and lower arm
        drive.followTrajectorySequence(seq_2);
    }
}