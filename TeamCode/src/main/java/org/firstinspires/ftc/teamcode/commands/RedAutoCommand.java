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
                        .addTemporalMarker(new PivotCommand(pivot,Math.toRadians(20))::schedule)
                        .back(21) //move back
                        .strafeLeft(55)//move right
                        .forward(6)
                        .addTemporalMarker(new IntakeCommand(intake, -1).withTimeout(900)::schedule)
                        .back(5)
                        .addTemporalMarker(new PivotCommand(pivot,0)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 1).withTimeout(500)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new PivotCommand(pivot,Math.toRadians(-20))::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(0.5)
                        .forward(9)
                        .addTemporalMarker(new PivotCommand(pivot,Math.toRadians(20))::schedule)
                        .waitSeconds(0.3)
                        .addTemporalMarker(new IntakeCommand(intake, -1)::schedule)
                        .waitSeconds(1.0)
                        .back(35)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .strafeLeft(60)
                        .forward(60)
                        .build()

                )
        );
    }

}
