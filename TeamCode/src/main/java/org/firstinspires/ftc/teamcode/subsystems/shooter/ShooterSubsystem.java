package org.firstinspires.ftc.teamcode.subsystems.shooter;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ShooterSubsystem extends SubsystemBase {

    private final MotorEx m_shooter;
    private double RPM;

    public ShooterSubsystem(@NonNull HardwareMap hardwareMap) {
        RPM = 0.0;
        m_shooter = hardwareMap.get(MotorEx.class, "shooter");
        m_shooter.setRunMode(Motor.RunMode.VelocityControl);
        m_shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_shooter.setInverted(false);
        m_shooter.setVeloCoefficients(0.05, 0.01, 0.31);
        m_shooter.setFeedforwardCoefficients(0.92, 0.47);

    }

    public void setRPM(double setpoint) {
        RPM = setpoint;
        m_shooter.set(RPM / 60 * m_shooter.getCPR()); //converts rpm to rotations per second, multiplies by ticks per rotation to get target velocity in ticks per second
    }

    public double getRPMSetpoint() {
        return RPM;
    }

    public double getRPM() {
        return m_shooter.getVelocity() * 60 / m_shooter.getCPR();
    }
}
