package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

import java.util.function.DoubleSupplier;


public class ClawCommand extends CommandBase {

    private final ClawSubsystem m_claw;
    private double m_angle;

    public ClawCommand(ClawSubsystem claw, double angle){
        m_claw = claw;
        m_angle = angle;

        addRequirements(m_claw);
    }

    @Override
    public void execute(){
        m_claw.turnToAngle(m_angle);
    }

}