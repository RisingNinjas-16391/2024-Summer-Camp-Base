package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
    public BlueAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, OuttakeSubsystem outtake) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(50)
                        .strafeRight(54)
                        .forward(43)
                        .addTemporalMarker(new SequentialCommandGroup(
                                new OuttakeCommand(outtake, () -> 312.0).withTimeout(3000),
                                new OuttakeCommand(outtake, () -> 0.0).withTimeout(100)
                        )::schedule)
                        .back(5)
                        .addTemporalMarker(new SequentialCommandGroup(
                                new IntakeCommand(intake, () -> 312.0).withTimeout(100),
                                new OuttakeCommand(outtake, () -> 312.0).withTimeout(100)
                        )::schedule)
                        .forward(2.25)
                        .waitSeconds(3)
                        .addTemporalMarker(new SequentialCommandGroup(
                                new IntakeCommand(intake, () -> 0.0).withTimeout(100),
                                new OuttakeCommand(outtake, () -> 0.0).withTimeout(100)
                        )::schedule)
                        .back(49)
                        .strafeRight(60)
                        .forward(72)
                        .build()
                )
        );
    }
}
