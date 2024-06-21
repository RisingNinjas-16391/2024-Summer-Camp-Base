package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;

import java.util.function.DoubleSupplier;


public class IntakeCommand extends CommandBase {

    private final SingleMotorSubsystem m_intake;
    private DoubleSupplier m_power;

    public IntakeCommand(SingleMotorSubsystem intake, DoubleSupplier power){
        m_intake = intake;
        m_power = power;

        addRequirements(m_intake);
    }

    public IntakeCommand(SingleMotorSubsystem intake, double power){
        m_intake = intake;
        m_power = () -> power;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        m_intake.setPower(m_power.getAsDouble());
    }

    @Override
    public void end(boolean cancelled) {
        m_intake.setPower(0);
    }
}