package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.claw.ExtensionSubsystem;

//import org.firstinspires.ftc.teamcode.subsystems.extension.ExtensionSubsystem;

import java.util.function.DoubleSupplier;


public class ExtensionCommand extends CommandBase {

    private final ExtensionSubsystem m_extension;
    private double m_power;

    public ExtensionCommand(ExtensionSubsystem extend, double power){
        m_extension = extend;
        m_power = power;

        addRequirements(m_extension);
    }

    @Override
    public void execute(){
        m_extension.turnToAngle(m_power);
    }

}