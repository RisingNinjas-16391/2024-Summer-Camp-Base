package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw2Subsystem extends SubsystemBase {
    private final SimpleServo m_claw2;

    public Claw2Subsystem(HardwareMap hwMap) {
        m_claw2 = new SimpleServo(hwMap, "claw2", -40,
                180, AngleUnit.DEGREES);
        turnToAngle(-40);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Claw2");
        telemetry.addData("Angle", m_claw2.getAngle());
    }

    public void turnToAngle(double angle) {
        m_claw2.turnToAngle(angle);

    }

}