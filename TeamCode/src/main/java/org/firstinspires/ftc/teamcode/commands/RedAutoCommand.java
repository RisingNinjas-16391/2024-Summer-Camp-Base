package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot, WristSubsystem wrist, ClawSubsystem claw) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        /*.addTemporalMarker(new ClawCommand(claw, 150)::schedule)
                        .forward(40)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(27))::schedule)
                        .forward(14)
                        .waitSeconds(1)
                        .addTemporalMarker(new ClawCommand(claw, 95)::schedule)
                        .waitSeconds(2)*/
                        .addTemporalMarker(new ClawCommand(claw, 150)::schedule)
                        .strafeLeft(25)
                        .turn(Math.toRadians(-90))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(56))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 26)::schedule)
                        .back(28)
                        .strafeRight(8)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, 95)::schedule)
                        .waitSeconds(0.5)
                        .strafeLeft(40)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(0))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, -7)::schedule)
                        .forward(35)
                        .turn(Math.toRadians(-3))
                        .forward(10)
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, 150)::schedule)
                        .waitSeconds(0.5)
                        .turn(Math.toRadians(3))
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(56))::schedule)
                        .addTemporalMarker(new WristCommand(wrist, 26)::schedule)
                        .back(39)
                        .strafeRight(26\]{+})
                        .waitSeconds(0.5)
                        .addTemporalMarker(new ClawCommand(claw, 95)::schedule)
                        .waitSeconds(0.5)
                        .forward(20)
                        .forward(45)
                        .turn(Math.toRadians(90))
                        .waitSeconds(0.5)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(0))::schedule)
                        .back(90)
                        .build()
                )
        );
    }

}
