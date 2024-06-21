package org.firstinspires.ftc.teamcode.subsystems.outtake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class OuttakeSubsystem extends SubsystemBase {

    private final MotorEx m_outtake;

    public OuttakeSubsystem(@NonNull HardwareMap hardwareMap) {
        m_outtake = new MotorEx(hardwareMap, "outtake");
        m_outtake.setRunMode(Motor.RunMode.VelocityControl);
        m_outtake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_outtake.setInverted(true);
        m_outtake.setVeloCoefficients(0.05, 0.01, 0.31);
        m_outtake.setFeedforwardCoefficients(0.92, 0.47);

    }

    public void setRPM(double setpoint) {
        m_outtake.set(setpoint / 60 * m_outtake.getCPR()); //converts rpm to rotations per second, multiplies by ticks per rotation to get target velocity in ticks per second
    }

}
