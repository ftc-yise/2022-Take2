package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.ledLights;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.tensorFlow;


@Autonomous(name = "Auto Blue Right Low", group = "Linear Opmode")
public class AutonomousBlueRightTestForLow extends LinearOpMode {

    public float endLocation_X = 0;
    public float endLocation_Y = 16;
    public float endHeading_Z = 90;
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
        //tensorFlow tensor = new tensorFlow(hardwareMap);
        tfod = new tensorFlow(hardwareMap);

        // create instance of YISE drive class - for autoCenterLoop() only
        yiseDrive = new mecanumDrive(hardwareMap);

        // create instance of YISE lift arm class
        arm = new liftArm(hardwareMap);

        // create instance of YISE led lights class
        leds = new ledLights(hardwareMap);
        leds.setLed(ledLights.ledStates.RED);

        // set variable for holding the signal beacon detection for end placment
        tfod.initVuforia();
        tfod.initTfod();

        int coneNumber = 3;

        while (!opModeIsActive()) {
            coneNumber = tfod.readCone();
            if (coneNumber == 1) {
                leds.setLed(ledLights.ledStates.RED);
            } else if (coneNumber == 2) {
                leds.setLed(ledLights.ledStates.GREEN);
            } else if (coneNumber == 3) {
                leds.setLed(ledLights.ledStates.BLUE);
            }
            telemetry.addData("Cone: ", coneNumber);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // turn off tensorFlow
        tfod.disable();

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-38, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // disabling cone read after start
        //coneNumber = tfod.readCone();

        if (coneNumber == 3){
            endLocation_X = -66;
            endLocation_Y = 14;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.BLUE);
        } else if (coneNumber == 2){
            endLocation_X = -40;
            endLocation_Y = 16;
            endHeading_Z = 90;
            leds.setLed(ledLights.ledStates.GREEN);
        } else if (coneNumber == 1){
            endLocation_X = -15;
            endLocation_Y = 16;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.RED);
        }

        // Sequence 1 is start of program ending at cone pickup.
        TrajectorySequence startpath = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(23)
                .lineToLinearHeading(new Pose2d(-59,20, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-48, 14, Math.toRadians(180)))
                .addDisplacementMarker(30, () -> {
                    arm.getConeFromStack(stackPosition);
                    arm.openGrabber();
                })
                .forward(16)
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
                .lineToLinearHeading(new Pose2d(-51, 13, Math.toRadians(90)))
                .forward(3)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.05125)
                .back(7)
                .build();
        Pose2d scorePose = scorecone.end();

        TrajectorySequence grabcone = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d(-50, 14, Math.toRadians(180)))
                .addDisplacementMarker(1,() -> {
                    arm.getConeFromStack(stackPosition);
                })
                .forward(15)
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
        telemetry.addData("cone#", coneNumber);
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