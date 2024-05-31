package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;


public class ShooterCommand extends CommandBase {

    private final ShooterSubsystem m_shooter;
    private DoubleSupplier m_RPM;

    public ShooterCommand(ShooterSubsystem intake, DoubleSupplier RPM){
        m_shooter = intake;
        m_RPM = RPM;

        addRequirements(m_shooter);
    }

    @Override
    public void execute(){
        m_shooter.setRPM(m_RPM.getAsDouble());
    }
}