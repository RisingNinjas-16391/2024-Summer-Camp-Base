package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(25)
                        .strafeLeft(55)
                        .forward(10)
                        .addTemporalMarker(new IntakeCommand(intake, 2)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .forward(10)
                        .addTemporalMarker(new IntakeCommand(intake, 2)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(55)
                        .strafeLeft(53)
                        .forward(75)
                        .build()
                )

        );
    }
}
