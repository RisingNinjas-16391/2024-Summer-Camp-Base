package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;


public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem m_intake;
    private DoubleSupplier m_RPM;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier RPM){
        m_intake = intake;
        m_RPM = RPM;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        m_intake.setRPM(m_RPM.getAsDouble());
    }
}