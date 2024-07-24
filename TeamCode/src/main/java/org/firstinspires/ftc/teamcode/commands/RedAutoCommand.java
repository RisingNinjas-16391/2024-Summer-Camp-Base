package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot, ClawSubsystem claw, WristSubsystem wrist) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here1`
                        .waitSeconds(5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-20)).withTimeout(200)::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 120).withTimeout(200)::schedule)
                        .back(49)
                        .addTemporalMarker(new ClawCommand(claw, -20).withTimeout(100)::schedule)
                        .waitSeconds(.3)
                        .strafeLeft(19)
                        .addTemporalMarker(new WristCommand(wrist, 0).withTimeout(200)::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(22)).withTimeout(800)::schedule)
                        .forward(25.5)
                        .waitSeconds(.1)
                        .forward(1.5)
                        .waitSeconds(.4)
                        //grabs cone
                        .addTemporalMarker(new ClawCommand(claw, 25).withTimeout(100)::schedule)
                        .waitSeconds(.4)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-15)).withTimeout(800)::schedule)
                        .waitSeconds(.4)
                        .addTemporalMarker(new WristCommand(wrist, 120).withTimeout(200)::schedule)
                        .back(26)
                        .strafeRight(32)
                        .addTemporalMarker(new ClawCommand(claw, -20).withTimeout(200)::schedule)
                        .waitSeconds(.4)
                        .addTemporalMarker(new PivotCommand(pivot, 0).withTimeout(200)::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 0).withTimeout(200)::schedule)
                        .strafeLeft(7)
                        .forward(50)
                        .turn(Math.toRadians(90))
                        .strafeRight(10)
                        .back(65)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(22)).withTimeout(800)::schedule)
                        /*.addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-18)).withTimeout(200)::schedule)
                        .forward(48)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(-14)).withTimeout(200)::schedule)
                        .addTemporalMarker(new ClawCommand(claw, -20).withTimeout(200)::schedule)
                        .waitSeconds(.5)
                        .turn(Math.toRadians(180))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(0))::schedule)
                        .strafeLeft(20)
                        .forward(29)*/
                        .build()

                )
        );
    }

}
