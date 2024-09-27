package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .forward(53)
                        .turn(90)
                        .addTemporalMarker(new PivotCommand(pivot,140)::schedule)
                        .back(40)
                        .addTemporalMarker(new IntakeCommand(intake,-1)::schedule)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new IntakeCommand(intake,0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot,0)::schedule)
                        .forward(30)
                        .addTemporalMarker(new IntakeCommand(intake,1)::schedule)
                        .waitSeconds(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(1)
                        .forward(12)

                        .waitSeconds(0.5)
                        .addTemporalMarker(new IntakeCommand(intake,0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot,140)::schedule)
                        .back(54)
                        .addTemporalMarker(new IntakeCommand(intake,-1)::schedule)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new IntakeCommand(intake,0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot,30)::schedule)
                        .forward(37)
                        .strafeLeft(65)
                        .back(80)


                        /*.addTemporalMarker(new PivotCommand(pivot, 30)::schedule)
                        .forward(10)
                        .turn((90))
                        .strafeLeft(10)
                        .addTemporalMarker(new IntakeCommand(intake,-1)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake,0)::schedule)*/
                        .build()

                )
        );
    }

}
