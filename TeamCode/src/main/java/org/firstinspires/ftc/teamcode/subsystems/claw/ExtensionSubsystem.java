package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ExtensionSubsystem extends SubsystemBase {
    private final CRServo m_extension;
    private String ID;

    public ExtensionSubsystem(HardwareMap hwMap, String ID) {
        this.ID = ID;
        m_extension = new CRServo(hwMap, ID);

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Extension");
    }

    public void turnToAngle(double power) {
        m_extension.set(power);

    }

}