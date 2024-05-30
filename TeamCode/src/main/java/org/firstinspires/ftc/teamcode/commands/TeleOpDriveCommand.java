package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drive;

    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier omega;

    public TeleOpDriveCommand(final DrivetrainSubsystem drive, final DoubleSupplier y, final DoubleSupplier x, final DoubleSupplier omega) {
        this.drive = drive;

        this.y = y;
        this.x = x;
        this.omega = omega;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setHeading(drive.getHeading());
    }

    @Override
    public void execute() {
//        drive.setDrivePowerHeadingPID(new Pose2d(x.getAsDouble(), -y.getAsDouble(), -omega.getAsDouble()));
        drive.setDrivePower(new Pose2d(-x.getAsDouble(), y.getAsDouble(), -omega.getAsDouble()));
    }
}
