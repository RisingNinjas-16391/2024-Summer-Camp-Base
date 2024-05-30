package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.trajectorysequence.TrajectorySequence;

public class FollowTrajectoryCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final TrajectorySequence trajectory;

    public FollowTrajectoryCommand(DrivetrainSubsystem drivetrain, TrajectorySequence trajectory){
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize(){
        drivetrain.runTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drivetrain.update();
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.isBusy();
    }

}