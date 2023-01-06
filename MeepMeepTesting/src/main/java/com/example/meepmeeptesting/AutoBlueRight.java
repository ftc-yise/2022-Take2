package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoBlueRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.44, 49.44, Math.toRadians(180), Math.toRadians(180), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(90)))
                                .strafeRight(-12)
                                .splineToConstantHeading(new Vector2d(-48, 14), Math.toRadians(90))
                                .addDisplacementMarker(20, () -> {
                                    //arm.getTopCone();
                                })
                                .addDisplacementMarker(20, () -> {
                                    //arm.openGrabber();
                                })
                                .turn(Math.toRadians(90))
                                .forward(10)
                                .addTemporalMarker(() -> {
                                    //yiseDrive.autoCenterLoop(mecanumDrive.centerModes.CONE);
                                })
                                .addTemporalMarker(() -> {
                                    //arm.closeGrabber();
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(() -> {
                                    //arm.setPoleHeight(liftArm.Heights.HIGH);
                                })

                                //SEQ_2
                                .lineToConstantHeading(new Vector2d(-11, 14))
                                .turn(Math.toRadians(-140))
                                .forward(8)
                                .addTemporalMarker(() -> {
                                    //yiseDrive.autoCenterLoop(mecanumDrive.centerModes.POLE);
                                })
                                .waitSeconds(.2)
                                .addTemporalMarker(() -> {
                                    //arm.openGrabber();
                                })
                                .waitSeconds(.4)
                                .back(8)

                                //END SEQ
                                /* if (coneNumber == 3){
                                     endLocation_X = -58;
                                     endLocation_Y = 14;
                                     endHeading_Z = 180;
                                 } else if (coneNumber == 2){
                                     endLocation_X = -34;
                                     endLocation_Y = 16;
                                     endHeading_Z = 0;
                                 } else if (coneNumber == 1){
                                     endLocation_X = -12;
                                     endLocation_Y = 16;
                                     endHeading_Z = 0;
 }*/
                                //.lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y,  Math.toRadians(endHeading_Z)))
                                //.lineToLinearHeading(new Pose2d(-58, 14, Math.toRadians(-0)))  //location = 3
                                .lineToLinearHeading(new Pose2d(-34, 16, Math.toRadians(0))) // location = 2
                                //.lineToLinearHeading(new Pose2d(-12, 16, Math.toRadians(0)))  //location = 1
                                .addTemporalMarker(() ->{
                                    //arm.closeGrabber();
                                })
                                .addTemporalMarker(() ->{
                                    //arm.returnToBottom();
                                })
                                .turn(Math.toRadians(90))

                                //to see end point in meep meep
                                .waitSeconds(5)
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