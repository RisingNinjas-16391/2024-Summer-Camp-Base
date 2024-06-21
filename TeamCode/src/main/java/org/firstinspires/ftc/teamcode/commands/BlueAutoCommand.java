package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;

public class BlueAutoCommand extends SequentialCommandGroup {
    public BlueAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(40)
                        .strafeRight(52)
                        .forward(50)
                        .addTemporalMarker(new IntakeCommand(intake, 2)::schedule)
                        .waitSeconds(2)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(5)
                        .forward(50)
                        .addTemporalMarker(new IntakeCommand(intake, 2)::schedule)
                        .waitSeconds(2)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(55)
                        .strafeRight(70)
                        .forward(85)
                        .build()

                )
        );
    }

}
