package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.apache.commons.math3.analysis.function.Sin;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import java.util.Calendar;

public class BlueAutoCommand extends SequentialCommandGroup {
public BlueAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot, WristSubsystem wrist, ClawSubsystem claw) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        // sample commands
                        .addTemporalMarker(new ClawCommand(claw, 150)::schedule)
                        .strafeRight(25)
                        .turn(Math.toRadians(90))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(56))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 26)::schedule)
                        .back(39)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, 95)::schedule)
                        .waitSeconds(0.5)
                        .strafeRight(40)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(0))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, -7)::schedule)
                        .forward(35)
                        .turn(Math.toRadians(-17))
                        .forward(17)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, 150)::schedule)
                        .waitSeconds(0.5)
                        .turn(Math.toRadians(17))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(56))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 26)::schedule)
                        .back(46)
                        .strafeLeft(29)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, 95)::schedule)
                        .waitSeconds(0.5)
                        .forward(20)
                        .forward(45)
                        .turn(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(0))::schedule)
                        .back(90)
                       // .addTemporalMarker(new)

                        .build()
                )
        );
    }
}
