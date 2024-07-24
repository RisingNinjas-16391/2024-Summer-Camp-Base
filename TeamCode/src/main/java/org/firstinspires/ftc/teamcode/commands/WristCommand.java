package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;

public class WristCommand extends CommandBase {
    private final WristSubsystem m_wrist;
    private double m_angle;

    public WristCommand(WristSubsystem wrist, double angle){
        m_wrist = wrist;
        m_angle = angle;

        addRequirements(m_wrist);
    }

    @Override
    public void execute(){
        m_wrist.turnToAngle(m_angle);
    }

}
