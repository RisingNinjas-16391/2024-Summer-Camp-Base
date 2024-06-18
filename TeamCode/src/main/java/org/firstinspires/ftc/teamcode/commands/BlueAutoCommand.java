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
                        .back(21) //move back
                        .strafeRight(44)//move right
                        .forward(5)
                        .build()

                ),

                new IntakeCommand(intake, 1),

                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(35)//move back
                        .strafeRight(57)//move right
                        .forward(75)//move forward
                        .build()
                )
        );
    }
}
