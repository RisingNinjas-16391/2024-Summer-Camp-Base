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
                        .back(35)
                        .strafeRight(55)
                        .forward(24)
                        .back(9)
                        .forward(7)
                        .back(42.5)
                        .strafeRight(46.5)
                        .forward(60)
                        .build()
                        // Hello, World
                )
        );
    }
}
