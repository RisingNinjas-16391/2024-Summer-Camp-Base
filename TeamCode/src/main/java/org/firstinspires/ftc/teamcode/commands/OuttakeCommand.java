package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;

import java.util.function.DoubleSupplier;


public class OuttakeCommand extends CommandBase {

    private final OuttakeSubsystem m_outtake;
    private DoubleSupplier m_RPM;

    public OuttakeCommand(OuttakeSubsystem outtake, DoubleSupplier RPM){
        m_outtake = outtake;
        m_RPM = RPM;

        addRequirements(m_outtake);
    }


    @Override
    public void execute(){
        m_outtake.setRPM(m_RPM.getAsDouble());
    }
}