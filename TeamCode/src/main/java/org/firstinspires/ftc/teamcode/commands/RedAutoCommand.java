package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.Claw2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, WristSubsystem wrist, PivotSubsystem pivot, ClawSubsystem claw, Claw2Subsystem claw2) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .addTemporalMarker(new ParallelCommandGroup(
                                new ClawCommand(claw, 100),
                                new Claw2Command(claw2, -40))::schedule
                        )
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-10))::schedule)
                        .waitSeconds(0.5)
                        .forward(35)
                        //raise arm + wrist
                        .waitSeconds(0.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-21))::schedule)
                        //score
                        .forward(22)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ParallelCommandGroup(
                                new ClawCommand(claw, 0),
                                new Claw2Command(claw2, 0))::schedule)
                        //feed position
                        .waitSeconds(0.5)
                        .back(25)
                        .strafeRight(35)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-65))::schedule)
                        .waitSeconds(0.25)
                        .back(16)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ParallelCommandGroup(
                                new ClawCommand(claw, 100),
                                new Claw2Command(claw2, -40))::schedule)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-22))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 20)::schedule)
                        .forward(25)
                        .strafeLeft(32)
                        .forward(20)
                        .waitSeconds(0.25)
                        .addTemporalMarker(new ParallelCommandGroup(
                                new ClawCommand(claw, 0),
                                new Claw2Command(claw2, 0))::schedule)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-35))::schedule)
                        .back(65)
                        .strafeLeft(120)
                        .build()
                )
        );
    }

}
