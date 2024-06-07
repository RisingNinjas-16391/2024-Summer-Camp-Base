package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class IndexerCommand extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final DoubleSupplier power;

    public IndexerCommand(final IndexerSubsystem indexer, final DoubleSupplier power) {
        indexerSubsystem = indexer;
        this.power = power;

        addRequirements(indexerSubsystem);
    }

    public IndexerCommand(final IndexerSubsystem indexer, final double power) {
        this(indexer, () -> power);
    }

    @Override
    public void execute() {
        indexerSubsystem.setPower(power.getAsDouble());
    }
}