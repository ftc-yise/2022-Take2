package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.tensorFlow;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;


@Autonomous(name = "Auto Red Right", group = "Linear Opmode")
public class AutonomousRedRight extends LinearOpMode {
    mecanumDrive yiseDrive;
    SampleMecanumDrive drive;
    liftArm arm;
    tensorFlow tfod;

    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------

        // create instance of roadrunner drive class
        drive = new SampleMecanumDrive(hardwareMap);

        yiseDrive = new mecanumDrive(hardwareMap);

        tfod = new tensorFlow(hardwareMap);
        tfod.initVuforia();
        tfod.initTfod();

        // create instance of yise lift arm class
        arm = new liftArm(hardwareMap);

        waitForStart();
        if(isStopRequested()) return;

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //Drive from starting pos to stack
        TrajectorySequence driveForward = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                .addDisplacementMarker(10, () -> {
                    arm.openGrabber();
                    arm.getTopCone();
                })
                .forward(12)
                .build();

        //Drive with cone to pole
        TrajectorySequence driveToPole = drive.trajectorySequenceBuilder(driveForward.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)))
                .addDisplacementMarker(() -> {
                    yiseDrive.autoCenterLoop(mecanumDrive.centerModes.POLE);
                })
                .forward(8)
                .build();

        //Drive back to stack to get another cone
        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(driveToPole.end())
                .back(8)
                .addDisplacementMarker(() -> {
                    arm.openGrabber();
                    arm.getTopCone();
                })
                .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(0)))
                .build();

        //Finishing positions
        TrajectorySequence driveTo1pos = drive.trajectorySequenceBuilder(driveToPole.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(0)))
                .build();
        TrajectorySequence driveTo2pos = drive.trajectorySequenceBuilder(driveToPole.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                .build();

        //Sense cones
        int cone = tfod.readCone();
        telemetry.addData("Cone: ", cone);
        telemetry.update();
        //Drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(driveForward);

        //Run method to pick up cone and drop it on pole
        coneLoop(driveToPole);

        //Drive back to get another cone
        drive.followTrajectorySequence(driveToStack);

        //Run method to pick up cone and drop it on pole
        coneLoop(driveToPole);

        //Drive to right position based on Tensorflow input
        if (cone == 1) {
            drive.followTrajectorySequence(driveTo1pos);
        } else if (cone == 2) {
            drive.followTrajectorySequence(driveTo2pos);
        } else {
            drive.followTrajectorySequence(driveToStack);
        }

    }

    public void coneLoop(TrajectorySequence poleTrajectory) {
        //Auto center on cones
        yiseDrive.autoCenterLoop(mecanumDrive.centerModes.CONE);

        //Grab and lift cone
        arm.closeGrabber();
        sleep(200);
        arm.setPoleHeight(liftArm.Heights.HIGH);

        //Drive to pole
        drive.followTrajectorySequence(poleTrajectory);
        //Center on pole and drop cone
        arm.openGrabber();
    }
}