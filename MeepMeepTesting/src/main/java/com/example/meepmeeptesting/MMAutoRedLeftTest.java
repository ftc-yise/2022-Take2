package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMAutoRedLeftTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.44, 49.44, Math.toRadians(180), Math.toRadians(180), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(270)))
                            .strafeRight(23)
                            .lineToLinearHeading(new Pose2d(-59,-20, Math.toRadians(270)))
                            .lineToLinearHeading(new Pose2d(-48, -14, Math.toRadians(180)))
                            .addDisplacementMarker(30, () -> {
                           // arm.getConeFromStack(stackPosition);
                           // arm.openGrabber();
                        })
                        .forward(15)
                        .addTemporalMarker(() -> {
                           // arm.closeGrabber();
                        })
                        .waitSeconds(0.125)
                        .addTemporalMarker(() -> {
                           // arm.setPoleHeight(liftArm.Heights.LOW  );
                        })
                        .back(7)

                        .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                        .forward(4)
                        .addTemporalMarker(() -> {
                           // arm.openGrabber();
                        })
                        .waitSeconds(0.125)
                        .back(6)

//cone 2

                        .lineToLinearHeading(new Pose2d(-50, -14, Math.toRadians(180)))
                        .addDisplacementMarker(1,() -> {
                        //     arm.getConeFromStack(stackPosition);
                        })
                        .forward(13)
                        .addTemporalMarker(() -> {
                                 //  arm.closeGrabber();
                        })
                        .waitSeconds(0.25)
                        .addTemporalMarker(() -> {
                               // arm.setPoleHeight(liftArm.Heights.LOW  );
                        })
                        .back(7)
                        .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                        .forward(4)
                        .addTemporalMarker(() -> {
                                   // arm.openGrabber();
                        })
                        .waitSeconds(0.125)
                        .back(6)

                                //cone 3

                                .lineToLinearHeading(new Pose2d(-50, -14, Math.toRadians(180)))
                                .addDisplacementMarker(1,() -> {
                                    //arm.getConeFromStack(stackPosition);
                                })
                                .forward(13)
                                .addTemporalMarker(() -> {
                                //    arm.closeGrabber();
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                  //  arm.setPoleHeight(liftArm.Heights.LOW  );
                                })
                                .back(7)

                                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                                .forward(4)
                                .addTemporalMarker(() -> {
                                   // arm.openGrabber();
                                })
                                .waitSeconds(0.125)
                                .back(6)
//cone 4
                                .lineToLinearHeading(new Pose2d(-50, -14, Math.toRadians(180)))
                                .addDisplacementMarker(1,() -> {
                                  //  arm.getConeFromStack(stackPosition);
                                })
                                .forward(13)
                                .addTemporalMarker(() -> {
                                    //arm.closeGrabber();
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                    //arm.setPoleHeight(liftArm.Heights.LOW  );
                                })
                                .back(7)

                                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                                .forward(4)
                                .addTemporalMarker(() -> {
                                    //arm.openGrabber();
                                })
                                .waitSeconds(0.125)
                                .back(6)
   // Cone 5
                                .lineToLinearHeading(new Pose2d(-50, -14, Math.toRadians(180)))
                                .addDisplacementMarker(1,() -> {
                                    //arm.getConeFromStack(stackPosition);
                                })
                                .forward(13)
                                .addTemporalMarker(() -> {
                                   // arm.closeGrabber();
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                   // arm.setPoleHeight(liftArm.Heights.LOW  );
                                })
                                .back(7)

                                .lineToLinearHeading(new Pose2d(-49, -13, Math.toRadians(270)))
                                .forward(4)
                                .addTemporalMarker(() -> {
                                 //   arm.openGrabber();
                                })
                                .waitSeconds(0.125)
                                .back(6)


 // End Position
                              //  .lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y,  Math.toRadians(endHeading_Z)))
                                .lineToLinearHeading(new Pose2d( -12, -16,  Math.toRadians(180)))
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