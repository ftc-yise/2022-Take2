package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.tensorFlow;


@Autonomous(name = "Auto Red Left", group = "Linear Opmode")
public class AutonomousRedLeft extends LinearOpMode {

    public float endLocation_X = 0;
    public float endLocation_Y = -16;
    public float endHeading_Z = -90;


    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------

        // create instance of RoadRunner drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // create instance of YISE tensorFlow class
        tensorFlow tensor = new tensorFlow(hardwareMap);

        // create instance of YISE drive class - for autoCenterLoop() only
        mecanumDrive yiseDrive = new mecanumDrive(hardwareMap);

        // create instance of YISE lift arm class
        liftArm arm = new liftArm(hardwareMap);

        // set variable for holding the signal beacon detection for end placment


        waitForStart();
        if (isStopRequested()) return;

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // example trajectory sequences
        // based on testing, there's no problem doing forward and strafeRight in a row

        // startpath_1:
        // Drive forward 20 inches (after first 10 inches, start raising arm to top cone height)
        // Then strafe right 20 inches and turn right 90 degrees
        //
        // This is an example of a "global" displacement marker, it actually doesn't matter where
        // it is placed in the order of the sequence. It will run after the bot has moved
        // a total of 10 inches.
        //
        // If you change the marker from 10 to 30, it will run in the middle of the stafeRight()
        // Note: 1 marker, when written in this syntax, can include multiple actions

        // Sequence 1 is start of program ending at cone pickup.
        TrajectorySequence startpath_1 = drive.trajectorySequenceBuilder(startPose)
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
                //.waitSeconds(2)

                .addTemporalMarker(() -> {
                    arm.closeGrabber();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.HIGH);
                })
                .waitSeconds(2)
                .build();


        TrajectorySequence scorecone_2 = drive.trajectorySequenceBuilder(startpath_1.end())
                .lineToConstantHeading(new Vector2d(-11, -14))
                .turn(Math.toRadians(135))
                .forward(5)
                .addTemporalMarker(() -> {
                    // yiseDrive.autoCenterLoop();
                })
                .waitSeconds(.2)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .back(6)
                .build();

        TrajectorySequence grabcone_3 = drive.trajectorySequenceBuilder(scorecone_2.end())
                .turn(Math.toRadians(-135))
                .lineToConstantHeading(new Vector2d(-52, -12))
                .addDisplacementMarker(20, () -> {
                    arm.getTopCone();
                    arm.downOneCone();
                })
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop();
                })
                .waitSeconds(.2)
                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                })
                .waitSeconds(.2)
                .build();

        //Need to add in additional trajectories becuase you can't repeat the motions and pick up a cone a different height

        TrajectorySequence endposition_4 = drive.trajectorySequenceBuilder(scorecone_2.end())
                .lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y, endHeading_Z))
               /* if (endLocation == 3) {
                    .lineToLinearHeading(new Pose2d(-12, -16, Math.toRadians(-90))) ;
                }
            else if (endLocation == 2) {
                .lineToLinearHeading(new Pose2d(-12, -34, Math.toRadians(-90)))
                }
                else if (endLocation == 1) {
                .lineToLinearHeading(new Pose2d(-12, -16, Math.toRadians(-90)))
                }
               */ .build();


        // run my trajectories in order

        // drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(startpath_1);
        drive.followTrajectorySequence(scorecone_2);
        drive.followTrajectorySequence(grabcone_3);
        drive.followTrajectorySequence(scorecone_2);
        drive.followTrajectorySequence(grabcone_3);
        drive.followTrajectorySequence(scorecone_2);
        drive.followTrajectorySequence(endposition_4);
        //Location 3 x =-12  y =-16
        //location 2 x =-34  y =-16
        //location 1 1x =-59  y =-16

    }
}