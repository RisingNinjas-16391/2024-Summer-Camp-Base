package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
public BlueAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .forward(57)
                        .turn(-90)
                        .addTemporalMarker(new PivotCommand(pivot,140)::schedule)
                        .back(38)
                        //.addTemporalMarker(new IntakeCommand(intake,-1)::schedule)
                        .waitSeconds(0.5)
                        //.addTemporalMarker(new IntakeCommand(intake,0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot,0)::schedule)
                        .forward(30)
                        //.addTemporalMarker(new IntakeCommand(intake,1)::schedule)
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
                        .waitSeconds(1)
                        //.addTemporalMarker(new IntakeCommand(intake,0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot,140)::schedule)
                        .back(43)
                       // .addTemporalMarker(new IntakeCommand(intake,-1)::schedule)
                        .waitSeconds(0.5)
                       // .addTemporalMarker(new IntakeCommand(intake,0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot,30)::schedule)
                        .forward(37)
                        .strafeRight(65)
                        .back(80)
                        .build()
                )
        );
    }
}
