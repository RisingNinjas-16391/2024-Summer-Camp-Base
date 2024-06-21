package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, PivotSubsystem pivot) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20)).withTimeout(100)::schedule)
                        .back(20)
                        .strafeLeft(53)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 1)::schedule)
                        .back(5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-30)).withTimeout(100)::schedule)
                        .addTemporalMarker(
                                new IntakeCommand(intake, -1)::schedule)
                        .forward(7)
                        .waitSeconds(1)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20)).withTimeout(1000)::schedule)
                        .forward(3)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(0.5)
                        .back(33)
                        .turn(Math.toRadians(-90))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-30)).withTimeout(100)::schedule)
                        .addTemporalMarker(
                                new IntakeCommand(intake, -1)::schedule)
                        .forward(35)
                        .waitSeconds(1)
                        .back(35)
                        .turn(Math.toRadians(90))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20)).withTimeout(100)::schedule)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 0)::schedule)
                        .forward(35)
                        .waitSeconds(0.5)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 1)::schedule)
                        .back(40)
                        .strafeLeft(50)
                        .forward(60)
                        .build()
                )
        );
    }
}
