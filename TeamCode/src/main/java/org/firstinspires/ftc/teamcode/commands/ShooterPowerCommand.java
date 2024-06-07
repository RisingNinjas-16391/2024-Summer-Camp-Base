package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterPowerCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier power;

    public ShooterPowerCommand(final ShooterSubsystem shooter, final DoubleSupplier power) {
        shooterSubsystem = shooter;
        this.power = power;

        addRequirements(shooterSubsystem);
    }

    public ShooterPowerCommand(final ShooterSubsystem shooter, final double power) {
        this(shooter, () -> power);
    }

    @Override
    public void execute() {
        shooterSubsystem.setPower(power.getAsDouble());
    }
//
//    @Override
//    public boolean isFinished() {
//        return !shooterSubsystem.isBusy();
//    }
}