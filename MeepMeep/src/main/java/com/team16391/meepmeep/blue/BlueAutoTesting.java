package com.team16391.meepmeep.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueAutoTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 15.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .back(15)
                                .strafeLeft(29)
                                .turn(Math.toRadians(180))
                                //shoot
                                .turn(Math.toRadians(180))
                                .forward(18)
                                //collect
                                .back(18)
                                .turn(Math.toRadians(180))
                                //shoot
                                .forward(50)
                                //collect
                                .back(50)
                                //shoot
                                .forward(35)
                                .strafeRight(40)
                                .back(70)


                                .build());

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}