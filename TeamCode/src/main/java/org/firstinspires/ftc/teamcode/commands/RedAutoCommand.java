package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, IntakeSubsystem outtake) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(30)
                        .strafeLeft(66)
                        .forward(20)
                        .addTemporalMarker(new IntakeCommand(outtake, 1)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(1.2)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .addTemporalMarker(new IntakeCommand(outtake, 0)::schedule)
                        .back(36) //40 if not 3 point
                        //here
                        .turn(Math.toRadians(-90))
                        .forward(62)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(.8)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(43)
                        .turn(Math.toRadians(90))
                        .forward(38)
                        .addTemporalMarker(new IntakeCommand(outtake, 1)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(.5)
                        .addTemporalMarker(new IntakeCommand(outtake, 0)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .back(40)
                        // to here
                        .strafeLeft(62)
                        .forward(65)
                        .build()
                )


        );
    }
}
