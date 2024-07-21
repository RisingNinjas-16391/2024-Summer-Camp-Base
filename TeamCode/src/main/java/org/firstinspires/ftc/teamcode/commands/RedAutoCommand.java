package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, IntakeSubsystem intake, PivotSubsystem pivot, ClawSubsystem claw, ColorSubsystem sensor) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(10)).withTimeout(100)::schedule)
                        .forward(47.5)
                        .waitSeconds(0.7)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(100))::schedule)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, Math.toRadians(0))::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-80))::schedule)
                        .waitSeconds(.8)
                        .back(20)
                        .strafeRight(20)
                        .back(10)
                        .waitSeconds(.1)
                        .back(3.7)
                        .addTemporalMarker(new ClawCommand(claw,45)::schedule)
                        .waitSeconds(.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-30))::schedule)
                        .forward(17)
                        .strafeLeft(29.9)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(70))::schedule)
                        .waitSeconds(1)
                        .forward(18)
                        .waitSeconds(.25)
                        .addTemporalMarker(new ClawCommand(claw, Math.toRadians(0))::schedule)
                        .waitSeconds(.5)
                        .back(15)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(20))::schedule)
                        .waitSeconds(.5)
                        .back(40)
                        .turn(Math.toRadians(90))
                        .strafeLeft(5)
                        .forward(60)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-80))::schedule)



                        .build()


                )

                        /*
                        .strafeRight(50)
                        .back(15)
                        .forward(2)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(135)).withTimeout(700)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 1)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-25)).withTimeout(2000)::schedule)
                        .waitSeconds(2)
                        .addTemporalMarker(new IntakeCommand(intake, -1)::schedule)
                        .forward(42)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, -.2)::schedule)
                        .back(52)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(135)).withTimeout(700)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, .8)::schedule)
                        .waitSeconds(1)
                        .addTemporalMarker(new IntakeCommand(intake, 0)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(50)).withTimeout(700)::schedule)
                        .forward(46)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(90)).withTimeout(700)::schedule)
                        .strafeRight(50)
                        .back(66)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(35)).withTimeout(700)::schedule)
                        .addTemporalMarker(new IntakeCommand(intake, -1)::schedule)
                        .forward(20)
                        .turn(Math.toRadians(-90))
                        .back(15)
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
                        .forward(60)*/


        );
    }
}
