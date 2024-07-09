package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {
    private final SimpleServo m_claw;

    public ClawSubsystem(HardwareMap hwMap) {
        m_claw = new SimpleServo(hwMap, "claw", 0,
                180, AngleUnit.DEGREES);
        turnToAngle(0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Claw");
        telemetry.addData("Angle", m_claw.getAngle());
    }

    public void turnToAngle(double angle) {
        m_claw.turnToAngle(angle);

    }

}