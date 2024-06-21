package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, PivotSubsystem pivot) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(30)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-70))::schedule)
                        .strafeLeft(55)
                        .forward(16)
                        .addTemporalMarker(new IntakeCommand(intake, -6)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(33)
                        .strafeLeft(49)
                        .forward(60)
                        .build()

                )
        );
    }

}
