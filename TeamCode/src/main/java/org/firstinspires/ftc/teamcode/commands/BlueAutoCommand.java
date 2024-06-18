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
                        // Add movements here
                        .forward(1)
                        .turn(Math.toRadians(90))
                        .strafeLeft(10)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(0))::schedule)
                        .waitSeconds(10)
                        .strafeRight(10)
                        .build()
                )

        );
    }
}
