package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ColorSubsystem;


public class ColorCommand extends CommandBase {

    private final ColorSubsystem m_ConeSensor;
    private int m_red;
    private int m_blue;
    private int m_green;

    public ColorCommand(ColorSubsystem color, int m_red){
        m_ConeSensor = color;
        addRequirements(m_ConeSensor);
    }

    //@Override
   //public void execute(){
    //    m_claw.turnToAngle(m_angle);
    }

