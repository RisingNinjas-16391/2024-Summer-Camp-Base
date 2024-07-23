package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.claw.Claw2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, PivotSubsystem pivot, ClawSubsystem claw, Claw2Subsystem claw2, WristSubsystem wrist) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .addTemporalMarker(new ParallelCommandGroup(
                                new ClawCommand(claw, 45),
                                new Claw2Command(claw2, 15)
                        )::schedule)
                        .addTemporalMarker(new PivotCommand(pivot, Math.toRadians(40))::schedule)
                        .back(60)
                        .build()

                )
        );
    }

}

//                          .forward(10)
//                        .turn(Math.toRadians(90))
//                        .strafeLeft(10)
//                           m_wristFront.whenPressed(new WristCommand(m_wristSubsystem, 125));
//                           new PivotCommand(m_pivotSubsystem, Math.toRadians(58)),

//                       .addTemporalMarker(new ParallelCommandGroup(
//                                new ClawCommand(claw, 45),
//                                new Claw2Command(claw2, 15)
//                        )::schedule)