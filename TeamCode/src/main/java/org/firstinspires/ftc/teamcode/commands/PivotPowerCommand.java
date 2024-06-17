package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class PivotPowerCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final DoubleSupplier power;
    private double currentAngle = 0;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static double speed = 1;

    public PivotPowerCommand(PivotSubsystem pivot, DoubleSupplier power) {
        this.pivot = pivot;
        this.power = power;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (Math.abs(power.getAsDouble()) < 0.1) {
            timer.reset();
            return;
        }
        pivot.setAngle(pivot.getAngle() + (power.getAsDouble() * timer.time() * speed));
    }

}