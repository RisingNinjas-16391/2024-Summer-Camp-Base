package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
public BlueAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, PivotSubsystem pivot, ClawSubsystem claw, WristSubsystem wrist) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        /*.addTemporalMarker(new PivotCommand(pivot, Math.toRadians(35)).withTimeout(100)::schedule)
                        .forward(20)
                        .strafeLeft(49)
                        .back(15)
                        .forward(2)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(135)).withTimeout(700)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, .8)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-25)).withTimeout(2000)::schedule)
                        .waitSeconds(2)
                        .addTemporalMarker(new IntakeCommand(intake, -1)::schedule)
                        .forward(45)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, -.2)::schedule)
                        .back(55)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(135)).withTimeout(700)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, .8)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(50)).withTimeout(700)::schedule)
                        .forward(46)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(90)).withTimeout(700)::schedule)
                        .strafeLeft(51)
                        .back(66)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20)).withTimeout(100)::schedule)
                        .back(20)
                        .strafeRight(53)
                        .forward(10)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 1)::schedule)
                        .back(10)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-30)).withTimeout(100)::schedule)
                        .addTemporalMarker(
                                new IntakeCommand(intake, -1)::schedule)
                        .forward(8)
                        .waitSeconds(1)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20)).withTimeout(1000)::schedule)
                        .forward(2)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(0.5)
                        .back(43)
                        .turn(Math.toRadians(90))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-30)).withTimeout(100)::schedule)
                        .addTemporalMarker(
                                new IntakeCommand(intake, -1)::schedule)
                        .forward(35)
                        .waitSeconds(1)
                        .back(35)
                        .turn(Math.toRadians(-90))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20)).withTimeout(100)::schedule)
                        .forward(38)
                        .waitSeconds(0.5)
                        .addTemporalMarker(
                                new IntakeCommand(intake, 1)::schedule)
                        .back(40)
                        .strafeRight(50)
                        .forward(65)*/
                        .build()
                )
        );
    }
}
