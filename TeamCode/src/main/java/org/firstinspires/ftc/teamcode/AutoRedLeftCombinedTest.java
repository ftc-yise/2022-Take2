package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.ledLights;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.tensorFlow;


@Autonomous(name = "Auto Red Left Combined", group = "Linear Opmode")
@Disabled
public class AutoRedLeftCombinedTest extends LinearOpMode {

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
        //tensorFlow tensor = new tensorFlow(hardwareMap);
        tensorFlow tfod = new tensorFlow(hardwareMap);

        // create instance of YISE drive class - for autoCenterLoop() only
        mecanumDrive yiseDrive = new mecanumDrive(hardwareMap);

        // create instance of YISE lift arm class
        liftArm arm = new liftArm(hardwareMap);

        ledLights leds = new ledLights(hardwareMap);

       // int coneNumber;

        // set variable for holding the signal beacon detection for end placment
        //tensor.initVuforia();
        //tensor.initTfod();
        tfod.initVuforia();
        tfod.initTfod();

        int coneNumber;
        leds.setLed(ledLights.ledStates.RED);

     /*   while (!isStarted()) {
            //coneNumber = tensor.readCone();
            coneNumber = tfod.readCone();

            telemetry.addData("Cone: ", coneNumber);
            telemetry.update();
        }
    */
        waitForStart();
        if (isStopRequested()) return;



        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //coneNumber = tensor.readCone();
       coneNumber = tfod.readCone();

        if (coneNumber == 1){
            endLocation_X = -61;
            endLocation_Y = -14;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.RED);
        } else if (coneNumber == 2){
            endLocation_X = -35;
            endLocation_Y = -16;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.GREEN);
        } else if (coneNumber == 3){
            endLocation_X = -12;
            endLocation_Y = -16;
            endHeading_Z = 180;
            leds.setLed(ledLights.ledStates.BLUE);
        }

        // Sequence 1 is start of program ending at cone pickup.
        TrajectorySequence cone1 = drive.trajectorySequenceBuilder(startPose)
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

                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                .forward(4)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.125)
                .back(6)
                .build();
        Pose2d stackPose = cone1.end();

        TrajectorySequence cone2 = drive.trajectorySequenceBuilder(stackPose)


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
                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                .forward(4)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.125)
                .back(6)

                .build();
        Pose2d scorePose = cone2.end();

        TrajectorySequence cone3 = drive.trajectorySequenceBuilder(scorePose)
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

                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                .forward(4)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.125)
                .back(6)
                .build();

        Pose2d scorePose2 = cone3.end();

        TrajectorySequence cone4 = drive.trajectorySequenceBuilder(scorePose2)
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

                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                .forward(4)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.125)
                .back(6)
                .build();
        Pose2d scorePose3 = cone4.end();

        TrajectorySequence cone5 = drive.trajectorySequenceBuilder(scorePose3)
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

                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                .forward(4)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(0.125)
                .back(6)
                .build();
        Pose2d scorePose4 = cone5.end();




        TrajectorySequence endposition = drive.trajectorySequenceBuilder(scorePose4)
                .lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y,  Math.toRadians(endHeading_Z)))
                //.lineToLinearHeading(new Pose2d( -35, -16,  Math.toRadians(180)))

                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                    arm.returnToBottom();
                })

                .build();


        // run my trajectories in order

        telemetry.addData("cone#", coneNumber);
        telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        //drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(cone1);
       // drive.followTrajectorySequence(scorecone);

        stackPosition = 4;
        drive.followTrajectorySequence(cone2);
        //drive.followTrajectorySequence(scorecone);

        stackPosition = 3;
        drive.followTrajectorySequence(cone3);
        //drive.followTrajectorySequence(scorecone);

        stackPosition = 2;
        drive.followTrajectorySequence(cone4);
        //drive.followTrajectorySequence(scorecone);

        stackPosition = 1;
        drive.followTrajectorySequence(cone5);
        //drive.followTrajectorySequence(scorecone);


        drive.followTrajectorySequence(endposition);
    }
}