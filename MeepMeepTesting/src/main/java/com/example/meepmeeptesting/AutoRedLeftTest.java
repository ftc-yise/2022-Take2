package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoRedLeftTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.44, 49.44, Math.toRadians(180), Math.toRadians(180), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(270)))
                                .strafeRight(12)
                                .splineToConstantHeading(new Vector2d(-48, -14), Math.toRadians(270))
                                .addDisplacementMarker(20, () -> {
                                    //arm.getTopCone();
                                })
                                .addDisplacementMarker(20, () -> {
                                    //arm.openGrabber();
                                })
                                .turn(Math.toRadians(-90))
                                .forward(10)
                                .addTemporalMarker(() -> {
                                   // yiseDrive.autoCenterLoop(mecanumDrive.centerModes.CONE);
                                })
                                //.waitSeconds(2)

                                .addTemporalMarker(() -> {
                                   // arm.closeGrabber();
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(() -> {
                                    //arm.setPoleHeight(liftArm.Heights.LOW);
                                })
                                //seq 2
                                .back(10)
                                .turn(Math.toRadians(90))
                                .addTemporalMarker(() -> {
                                    // arm.openGrabber();
                                })
                                .turn(Math.toRadians(-90))
                                .build()

                );
        myBot.setDimensions(13, 18);
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}