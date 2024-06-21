package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
    public BlueAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, IntakeSubsystem outtake) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(30)
                        .strafeRight(73)
                        .forward(20)
                        .addTemporalMarker(new IntakeCommand(outtake, 1)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(1.2)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .addTemporalMarker(new IntakeCommand(outtake, 0)::schedule)
                        .back(45) //40 if not 3 point
                        //here
                        .turn(Math.toRadians(90))
                        .forward(62)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(.8)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(42)
                        .turn(Math.toRadians(-90))
                        .forward(40)
                        .addTemporalMarker(new IntakeCommand(outtake, 1)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(.5)
                        .addTemporalMarker(new IntakeCommand(outtake, 0)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(45)
                        // to here
                        .strafeRight(62)
                        .forward(67)
                        .build()
                )

        );
    }
}
