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
                                .strafeRight(15)
                                .splineToConstantHeading(new Vector2d(-52, -12), Math.toRadians(270))
                                .turn(Math.toRadians(-90))
                                .waitSeconds(.2)
                                .waitSeconds(.2)
                                .waitSeconds(2)
                                //.splineToConstantHeading(new Vector2d(-11, -14), Math.toRadians(135))
                                .lineToConstantHeading(new Vector2d(-5, -20))
                                .turn(Math.toRadians(140))
                                /*.waitSeconds(2)
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(-52, -12))
                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(-11, -14), Math.toRadians(180))
                                .turn(Math.toRadians(135))
                                .waitSeconds(2)
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(-52, -12))
                                .waitSeconds(2)
                                .back(14)
                                .lineToConstantHeading(new Vector2d(-40, -40))*/
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