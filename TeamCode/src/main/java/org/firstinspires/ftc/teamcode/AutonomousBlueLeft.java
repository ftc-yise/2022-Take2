package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.tensorFlow;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.ledLights;


@Autonomous(name = "Auto Blue Left", group = "Linear Opmode")
public class AutonomousBlueLeft extends LinearOpMode {
    mecanumDrive yiseDrive;
    SampleMecanumDrive drive;
    liftArm arm;
    tensorFlow tfod;

    int cone;
    int loop = 1;

    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------

        // create instance of roadrunner drive class
        drive = new SampleMecanumDrive(hardwareMap);
        yiseDrive = new mecanumDrive(hardwareMap);
        ledLights leds = new ledLights(hardwareMap);

        tfod = new tensorFlow(hardwareMap);
        tfod.initVuforia();
        tfod.initTfod();

        // create instance of yise lift arm class
        arm = new liftArm(hardwareMap);

        leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        // ------------------------------------------------------------------------------------
        // INITIALIZE TRAJECTORIES
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(36, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Drive from starting pos to stack
        TrajectorySequence driveForward = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
                .addDisplacementMarker(10, () -> {
                    arm.openGrabber();
                    arm.getTopCone();
                })
                .forward(26)
                .back(1)
                .build();

        //Drive with cone to pole
        TrajectorySequence driveToPole = drive.trajectorySequenceBuilder(driveForward.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(48, 11, Math.toRadians(90)))
                .forward(5)
                .addDisplacementMarker(0.2, () -> {
                    arm.setPoleHeight(liftArm.Heights.LOW);
                })
                .addDisplacementMarker(3, () -> {
                    if (!arm.findBlue() || !arm.findRed()) {
                        telemetry.addData("Cone not collected, new height: ", 6-loop);
                        telemetry.update();
                        loop--;
                    }
                })
                .build();

        //Drive back to stack to get another cone
        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(driveToPole.end())
                .back(5)
                .addDisplacementMarker(() -> {
                    arm.openGrabber();
                    arm.getTopCone();
                    for (int i = 0; i < loop; i++) {
                        arm.downOneCone();
                    }
                    loop++;
                })
                .lineToLinearHeading(new Pose2d(48, 13, Math.toRadians(0)))
                .forward(14)
                .back(1)
                .build();

        //Finishing positions
        TrajectorySequence driveTo1pos = drive.trajectorySequenceBuilder(driveToPole.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(13, 12, Math.toRadians(90)))
                .forward(5)
                .build();
        TrajectorySequence driveTo2pos = drive.trajectorySequenceBuilder(driveToPole.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(90)))
                .forward(5)
                .build();

        //Sense cones
        while (!isStarted()) {
            sleep(2000);
            cone = tfod.readCone();

            telemetry.addData("Cone: ", cone);
            telemetry.update();
        }

        if(isStopRequested()) return;


        // -----------------------------------------------------------------------
        // START OF PROGRAM
        // -----------------------------------------------------------------------


        //Sense cones
        switch (cone) {
            case 1:
                leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case 2:
                leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case 3:
                leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
        }

        //Stop tensorflow from running
        tfod.tfod.shutdown();

        //Drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(driveForward);

        leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);

        //Run method to pick up cone and drop it on pole
        coneLoop(driveToPole);

        //Loop through pole and cone stack
        for (int i = 0; i < 2; i++) {
            //Drive back to get another cone
            drive.followTrajectorySequence(driveToStack);

            //Run method to pick up cone and drop it on pole
            coneLoop(driveToPole);
        }

        switch (cone) {
            case 1:
                leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case 2:
                leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case 3:
                leds.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
        }

        //Drive to right position based on Tensorflow input
        if (cone == 1) {
            drive.followTrajectorySequence(driveTo1pos);
        } else if (cone == 2) {
            drive.followTrajectorySequence(driveTo2pos);
        } else {
            drive.followTrajectorySequence(driveToStack);
        }
        arm.returnToBottom();

    }

    public void coneLoop(TrajectorySequence poleTrajectory) {
        //Grab and lift cone
        arm.closeGrabber();
        sleep(500);

        //Drive to pole
        drive.followTrajectorySequence(poleTrajectory);
        //Center on pole and drop cone
        arm.openGrabber();
        sleep(200);
    }
}