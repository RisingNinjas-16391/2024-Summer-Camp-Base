package com.team16391.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Shapes {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 15.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                // Add movements here
                             //   .forward(10)
                              //  .turn(Math.toRadians(90))
                                .splineTo(new Vector2d(-40, 40), Math.toRadians(3))
                                .splineTo(new Vector2d(40, -40), Math.toRadians(18))
                                .splineTo(new Vector2d(-6.9, 42.0), Math.toRadians(6.9420))
                                .splineToSplineHeading(new Pose2d(57, 43, Math.toRadians(69)), Math.toRadians(3.141592653589793238462643383279502))
                                .turn(Math.toRadians(180))
                                .forward(6.9)
                                .splineToSplineHeading(new Pose2d(20, 21, Math.toRadians(22)), Math.toRadians(23))
                                .splineToSplineHeading(new Pose2d(21, 69, Math.toRadians(42)), Math.toRadians(21.69420))
                                .turn(Math.toRadians(180.99999999999999999))
                                .forward(22.5201938638219039208176283910)
                                .splineToSplineHeading(new Pose2d(5.319009, 6.3715, Math.toRadians(5.18)), Math.toRadians(6.942021))
                                .splineToSplineHeading(new Pose2d(16.5, 19.73285, Math.toRadians(8.5)), Math.toRadians(12))
                                .splineToSplineHeading(new Pose2d(69, 23, Math.toRadians(18)), Math.toRadians(35))
                                .turn(Math.toRadians(180))
                                .forward(14.206921)
                                .splineToSplineHeading(new Pose2d(42, 66, Math.toRadians(45)), Math.toRadians(373))
                                .splineToSplineHeading(new Pose2d(11.087, 42.787, Math.toRadians(44.903)), Math.toRadians(16.196))
                                .splineToSplineHeading(new Pose2d(59.181, 15.420, Math.toRadians(26.3236)), Math.toRadians(36.774))
                                .splineToSplineHeading(new Pose2d(3, 5, Math.toRadians(7)), Math.toRadians(9))
                                .splineToSplineHeading(new Pose2d(4, 6, Math.toRadians(8)), Math.toRadians(10))
                                .splineToSplineHeading(new Pose2d(15, 15, Math.toRadians(15)), Math.toRadians(15))

















                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}