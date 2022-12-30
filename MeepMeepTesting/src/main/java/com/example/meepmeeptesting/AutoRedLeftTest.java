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
                                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(270))
                                .addDisplacementMarker(20, () -> {
                                   // arm.getTopCone();
                                })
                                .addDisplacementMarker(20, () -> {
                                   // coneGrabber.setPosition(Servo.MIN_POSITION);
                                })
                                .turn(Math.toRadians(-90))
                                .forward(12)
                                .addTemporalMarker(2, () -> {
                                    //yiseDrive.autoCenter();
                                })
                                //.waitSeconds(2)
                                .addTemporalMarker(2,() -> {
                                    //coneGrabber.setPosition(Servo.MAX_POSITION);
                                })
                                //.waitSeconds(2)
                                .addTemporalMarker(2, () -> {
                                    //arm.setPoleHeight(liftArm.Heights.HIGH);
                                })
                                //.waitSeconds(2)
                                //SEQ2
                                .lineToConstantHeading(new Vector2d(-11, -14))
                                .turn(Math.toRadians(135))
                                .forward(6)
                                .addTemporalMarker(.2,() -> {
                                   // yiseDrive.autoCenter();
                                })
                                //.waitSeconds(.2)
                                .addTemporalMarker(.2, () -> {
                                    //coneGrabber.setPosition(Servo.MIN_POSITION);
                                })
                                //SEQ3
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(-52, -12))
                                .addDisplacementMarker(20, () -> {
                                   // arm.getTopCone();
                                   // arm.downOneCone();
                                })
                                .addTemporalMarker(() -> {
                                    //yiseDrive.autoCenter();
                                })
                                //.waitSeconds(.2)
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