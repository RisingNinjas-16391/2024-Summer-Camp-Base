package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem;

import java.util.function.DoubleSupplier;
public class ElevatorVelocityCommand extends CommandBase {
    private final ElevatorSubsystem m_elevator;

    private final DoubleSupplier m_velocity;

    public ElevatorVelocityCommand(ElevatorSubsystem elevator, DoubleSupplier velocity) {
        m_elevator = elevator;
        m_velocity = velocity;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setVelocity(m_velocity.getAsDouble());
    }



}
