package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx m_intake;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap){
        m_intake = new MotorEx(hardwareMap, "intake");
        m_intake.setRunMode(Motor.RunMode.VelocityControl);
        m_intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_intake.setInverted(false);
        m_intake.setVeloCoefficients(0.05, 0.01, 0.31);
        m_intake.setFeedforwardCoefficients(0.92, 0.47);

    }

    public void setRPM(double setpoint) {
        m_intake.set(setpoint / 60 * m_intake.getCPR()); //converts rpm to rotations per second, multiplies by ticks per rotation to get target velocity in ticks per second
    }

}