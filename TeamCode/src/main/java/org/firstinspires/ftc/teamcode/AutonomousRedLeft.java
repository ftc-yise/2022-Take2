package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;


@Autonomous(name = "Auto Red Left", group = "Linear Opmode")
public class AutonomousRedLeft extends LinearOpMode {

    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------

        // create instance of roadrunner drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TensorFlow tensor = new TensorFlow(hardwareMap);

        // create instance of YISE drive class - for autoCenterLoop() only
        mecanumDrive yiseDrive = new mecanumDrive(hardwareMap);

        // create instance of yise lift arm class
        liftArm arm = new liftArm(hardwareMap);

        //Initialize TensorFlow
        tensor.initVuforia();
        tensor.initTfod();

        waitForStart();
        if(isStopRequested()) return;

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));
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
                .strafeRight(12)
                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(270))
                .addDisplacementMarker(20, () -> {
                    arm.getTopCone();
                })
                .addDisplacementMarker(20, () -> {
                    arm.openGrabber();
                })
                .turn(Math.toRadians(-90))
                //.forward(6)
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    arm.closeGrabber();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.HIGH);
                })
                .waitSeconds(2)
                .build();


        TrajectorySequence seq_2 = drive.trajectorySequenceBuilder(seq_1.end())
                .lineToLinearHeading(new Pose2d(0, -12, Math.toRadians(270)))
                .addTemporalMarker(() -> {
                   yiseDrive.autoCenterLoop();
                })
                .waitSeconds(.2)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .build();

        TrajectorySequence seq_3 = drive.trajectorySequenceBuilder(seq_2.end())
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                .addDisplacementMarker(20, () -> {
                    arm.getTopCone();
                    arm.downOneCone();
                })
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop();
                })
                .waitSeconds(.2)
                .build();
        // run my trajectories in order

        // drive to cone stack with arm at cone 5 height

        //Check for cone
        //1 is PurpleY, 2 is GreenS, 3 is RedE

        int cone = tensor.readCone();
        telemetry.addData("Cone read: ", cone);
        telemetry.update();

        /*drive.followTrajectorySequence(seq_1);
        drive.followTrajectorySequence(seq_2);
        drive.followTrajectorySequence(seq_3);
        drive.followTrajectorySequence(seq_2);*/
    }
}