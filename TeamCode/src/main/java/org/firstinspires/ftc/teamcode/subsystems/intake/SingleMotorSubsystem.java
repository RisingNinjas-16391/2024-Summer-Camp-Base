package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SingleMotorSubsystem extends SubsystemBase {
    private final DcMotorEx m_intake;
    private final DcMotorEx m_follower;

    public SingleMotorSubsystem(@NonNull HardwareMap hardwareMap){
        m_intake = hardwareMap.get(DcMotorEx.class, "pivot");
        m_follower = hardwareMap.get(DcMotorEx.class,"follower");
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intake.setDirection(DcMotorSimple.Direction.FORWARD);

        m_follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_follower.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Intake");
        telemetry.addData("Power", m_intake.getPower());
    }

    public void setPower(double power) {
        m_intake.setPower(power);
        m_follower.setPower(power);
    }

    public double getPower() {
        return m_intake.getPower();
    }

    public boolean isBusy() {
        return m_intake.isBusy();
    }

}