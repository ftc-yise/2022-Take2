package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.tensorFlow;
import org.firstinspires.ftc.teamcode.yise.ledLights;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "Auto Red Left Low", group = "Linear Opmode")
public class AutonomousRedLeftTestForLow extends LinearOpMode {

    public float endLocation_X = 0;
    public float endLocation_Y = -16;
    public float endHeading_Z = -90;

    // Used to keep track of which cone we are picking up off the stack
    public int stackPosition = 5;

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

        ledLights leds = new ledLights(hardwareMap);

        // set variable for holding the signal beacon detection for end placment
        tensor.initVuforia();
        tensor.initTfod();

        waitForStart();
        if (isStopRequested()) return;

        leds.setLed(ledLights.ledStates.RED);

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        int coneNumber = 3;
        coneNumber = tensor.readCone();

        if (coneNumber == 1){
            endLocation_X = -58;
            endLocation_Y = -14;
            endHeading_Z = 0;
        } else if (coneNumber == 2){
            endLocation_X = -35;
            endLocation_Y = -16;
            endHeading_Z = 0;
        } else if (coneNumber == 3){
            endLocation_X = -12;
            endLocation_Y = -16;
            endHeading_Z = -0;
        }

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
        TrajectorySequence startpath = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                .splineToConstantHeading(new Vector2d(-48, -14), Math.toRadians(270))
                .addDisplacementMarker(20, () -> {
                    arm.getConeFromStack(stackPosition);
                })
                .addDisplacementMarker(20, () -> {
                    arm.openGrabber();
                })
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop(mecanumDrive.centerModes.STACK);
                    yiseDrive.driveUntilClosed(arm);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.LOW  );
                })
                .back(5)
                .build();
        Pose2d stackPose = startpath.end();

        TrajectorySequence scorecone = drive.trajectorySequenceBuilder(stackPose)
                .lineToLinearHeading(new Pose2d(-46, -14, Math.toRadians(180)))
               // .splineToConstantHeading(new Vector2d(-46, -14), Math.toRadians(270))
                .turn(Math.toRadians(90))
                .forward(2)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .back(6)
                .build();
        Pose2d scorePose = scorecone.end();

        TrajectorySequence grabcone = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d(-48, -14, Math.toRadians(180)))
                .addDisplacementMarker(1,() -> {
                    arm.getConeFromStack(stackPosition);
                })
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop(mecanumDrive.centerModes.STACK);
                    yiseDrive.driveUntilClosed(arm);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.LOW  );
                })
                .back(5)
                .build();


        /*
        TrajectorySequence grabcone_4 = drive.trajectorySequenceBuilder(scorecone_2.end())
                .turn(Math.toRadians(-135))
                .lineToConstantHeading(new Vector2d(-52, -12))
                .addDisplacementMarker(20, () -> {
                    arm.getTopCone();
                    arm.downOneCone();
                    arm.downOneCone();
                })
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop(mecanumDrive.centerModes.CONE);
                })
                .waitSeconds(.2)
                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                })
                .waitSeconds(.5)
                .build();

        TrajectorySequence testWait_5 = drive.trajectorySequenceBuilder(scorecone_2.end())
                .waitSeconds(10)
                .build();
         */

        //Need to add in additional trajectories becuase you can't repeat the motions and pick up a cone a different height

        TrajectorySequence endposition_4 = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y,  Math.toRadians(endHeading_Z)))
                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                })
                .addTemporalMarker(() ->{
                    arm.returnToBottom();
                })
                .turn(Math.toRadians(-90))
                .build();


        // run my trajectories in order

        telemetry.addData("cone#", coneNumber);
        telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        //drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(startpath);
        drive.followTrajectorySequence(scorecone);
        telemetry.update();


        //stackPosition = 4;
        //drive.followTrajectorySequence(grabcone);
        //drive.followTrajectorySequence(scorecone);
        /*
        stackPosition = 3;
        drive.followTrajectorySequence(grabcone);
        drive.followTrajectorySequence(scorecone);

        stackPosition = 2;
        drive.followTrajectorySequence(endposition_4);
        */

        // drive.followTrajectorySequence(testWait_5);
        //Location 3 x =-12  y =-16
        //location 2 x =-34  y =-16
        //location 1 1x =-59  y =-16


    }
}