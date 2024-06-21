package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot, ClawSubsystem claw) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .addTemporalMarker(new ClawCommand(claw, 90)::schedule)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(30))::schedule)
                        .back(28)
                        .strafeLeft(62)
                        .forward(35)
                        .addTemporalMarker(new ClawCommand(claw, 0)::schedule)
                        .back(10)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-15))::schedule)
                        .addTemporalMarker(new ClawCommand(claw, 90)::schedule)
                        .forward(10)
                        .addTemporalMarker(new ClawCommand(claw, 0)::schedule)
                        .back(35)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(40))::schedule)
                        .strafeLeft(60)
                        .forward(75)
                        .build()

                )
        );
    }

}
