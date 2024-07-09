package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ExtensionSubsystem extends SubsystemBase {
    private final SimpleServo m_extension;

    public ExtensionSubsystem(HardwareMap hwMap) {
        m_extension = new SimpleServo(hwMap, "extension", 0,
                180, AngleUnit.DEGREES);
        turnToAngle(0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Extension");
        telemetry.addData("Angle", m_extension.getAngle());
    }

    public void turnToAngle(double angle) {
        m_extension.turnToAngle(angle);

    }

}