package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.claw.Claw2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

import java.util.function.DoubleSupplier;


public class Claw2Command extends CommandBase {

    private final Claw2Subsystem m_claw2;
    private double m_angle;

    public Claw2Command(Claw2Subsystem claw2, double angle){
        m_claw2 = claw2;
        m_angle = angle;

        addRequirements(m_claw2);
    }

    @Override
    public void execute(){
        m_claw2.turnToAngle(m_angle);
    }

}