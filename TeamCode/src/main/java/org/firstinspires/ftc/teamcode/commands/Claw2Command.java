package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.claw.Claw2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;


public class Claw2Command extends CommandBase {

    private final Claw2Subsystem m_claw;
    private double m_angle;

    public Claw2Command(Claw2Subsystem claw, double angle){
        m_claw = claw;
        m_angle = angle;

        addRequirements(m_claw);
    }

    @Override
    public void execute(){
        m_claw.turnToAngle(m_angle);
    }

}