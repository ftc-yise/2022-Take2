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
    tensorFlow tfod;
    liftArm arm;
    SampleMecanumDrive drive;
    mecanumDrive yiseDrive;
    ledLights leds;

    // Used to keep track of which cone we are picking up off the stack
    public int stackPosition = 5;

    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------

        // create instance of RoadRunner drive class
        drive = new SampleMecanumDrive(hardwareMap);

        // create instance of YISE tensorFlow class
        tfod = new tensorFlow(hardwareMap);
        tfod.initVuforia();
        tfod.initTfod();

        // create instance of YISE drive class - for autoCenterLoop() only
        yiseDrive = new mecanumDrive(hardwareMap);

        // create instance of YISE lift arm class
        arm = new liftArm(hardwareMap);

        // create instance of YISE led light class
        leds = new ledLights(hardwareMap);
        leds.setLed(ledLights.ledStates.RED);

        int coneNumber = 3;

        while (!isStarted()) {
            sleep(1000);
            coneNumber = tfod.readCone();
            telemetry.addData("Cone: ", coneNumber);
            telemetry.update();
        }

        if (isStopRequested()) return;

        // turn off tensorFlow
        tfod.disable();

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        if (coneNumber == 1) {
            endLocation_X = -61;
            endLocation_Y = -14;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.RED);
        } else if (coneNumber == 2) {
            endLocation_X = -35;
            endLocation_Y = -16;
            endHeading_Z = -90;
            leds.setLed(ledLights.ledStates.GREEN);
        } else if (coneNumber == 3) {
            endLocation_X = -15;
            endLocation_Y = -16;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.BLUE);
        }

        // Sequence 1 is start of program ending at cone pickup.
        TrajectorySequence startpath = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(23)
                .lineToLinearHeading(new Pose2d(-59,-20, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-48, -14, Math.toRadians(180)))
                .addDisplacementMarker(30, () -> {
                    arm.getConeFromStack(stackPosition);
                    arm.openGrabber();
                })
                .forward(15)
                .addTemporalMarker(() -> {
                    arm.closeGrabber();
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.LOW  );
                })
                .back(7)
                .build();
        Pose2d stackPose = startpath.end();

        TrajectorySequence scorecone = drive.trajectorySequenceBuilder(stackPose)
                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                .forward(4)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.125)
                .back(6)
                .build();
        Pose2d scorePose = scorecone.end();

        TrajectorySequence grabcone = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d(-50, -14, Math.toRadians(180)))
                .addDisplacementMarker(1,() -> {
                    arm.getConeFromStack(stackPosition);
                })
                .forward(13)
                .addTemporalMarker(() -> {
                    arm.closeGrabber();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.LOW  );
                })
                .back(7)
                .build();

        TrajectorySequence endposition = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y,  Math.toRadians(endHeading_Z)))
                //.lineToLinearHeading(new Pose2d( -35, -16,  Math.toRadians(270)))
                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                })
                .addTemporalMarker(() ->{
                    arm.returnToBottom();
                })
                .build();

        // run my trajectories in order
        //telemetry.addData("cone#", coneNumber);
        telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        //drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(startpath);
        drive.followTrajectorySequence(scorecone);

        stackPosition = 4;
        drive.followTrajectorySequence(grabcone);
        drive.followTrajectorySequence(scorecone);

        stackPosition = 3;
        drive.followTrajectorySequence(grabcone);
        drive.followTrajectorySequence(scorecone);

        //stackPosition = 2;
        //drive.followTrajectorySequence(grabcone);
        //drive.followTrajectorySequence(scorecone);

        drive.followTrajectorySequence(endposition);
    }
}