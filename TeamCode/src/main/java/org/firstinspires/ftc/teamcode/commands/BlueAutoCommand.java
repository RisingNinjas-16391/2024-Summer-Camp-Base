package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
public BlueAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, PivotSubsystem pivot) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .back(33)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-70))::schedule)
                        .strafeRight(55)
                        .forward(11)
                        .addTemporalMarker(new IntakeCommand(intake, -6)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(35)
                        .strafeRight(49)
                        .forward(70)
                        .build()
                        // Hello, World
                )
        );
    }
}
