package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, SingleMotorSubsystem intake, SingleMotorSubsystem shooter) {
        addCommands(
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here1`
                        .forward(28)
                        .strafeRight(50)
                        //shoot
                        .forward(38)
                        //intake
                        //shoot
                        .back(38)
                        .turn(Math.toRadians(180))
                        .forward(15)
                        //intake
                        .back(39)
                        //shoot
                        .turn(Math.toRadians(180))
                        .strafeRight(49)
                        .back(59)
                        .build()

                )
        );
    }

}
