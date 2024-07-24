package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.apache.commons.math3.analysis.function.Sin;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
public BlueAutoCommand(DrivetrainSubsystem drive, WristSubsystem wrist, ClawSubsystem claw) {
    //SingleMotorSubsystem outtake;
    addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(30)
                        .strafeRight(50)
                        .forward(30)  
                        //.addTemporalMarker(new IntakeCommand(outtake, 1)::schedule)
                        .back(40)
                        .strafeRight(45)
                        .forward(85)
                        .build()
                )
        );
    }
}
