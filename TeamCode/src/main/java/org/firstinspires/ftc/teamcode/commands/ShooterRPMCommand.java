package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterRPMCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier RPM;

    public ShooterRPMCommand(final ShooterSubsystem shooter, final DoubleSupplier RPM) {
        shooterSubsystem = shooter;
        this.RPM = RPM;

        addRequirements(shooterSubsystem);
    }

    public ShooterRPMCommand(final ShooterSubsystem shooter, final double RPM) {
        this(shooter, () -> RPM);
    }

    @Override
    public void execute() {
        shooterSubsystem.setRPM(RPM.getAsDouble());
    }
//
//    @Override
//    public boolean isFinished() {
//        return !shooterSubsystem.isBusy();
//    }
}