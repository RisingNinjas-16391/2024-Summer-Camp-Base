package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem;



public class ElevatorPositionCommand extends CommandBase {


    private final ElevatorSubsystem m_elevator;

    private final double m_extension;

    public ElevatorPositionCommand(ElevatorSubsystem elevator, double extension) {
        m_elevator = elevator;
        m_extension = extension;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setGoal(m_extension);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.isAtPosition();


    }
}
