package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ColorSubsystem extends SubsystemBase {
    private final ColorSensor m_ConeSensor;

    public ColorSubsystem(HardwareMap hwMap) {
        m_ConeSensor = hwMap.get(ColorSensor.class, "color");
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("red Value",m_ConeSensor.red());

        telemetry.addData("blue Value",m_ConeSensor.blue());

        telemetry.addData("green Value",m_ConeSensor.green());
    }

    public void LEDon(boolean on) {
        m_ConeSensor.enableLed(on);

    }

    public boolean hasCone() {
        return m_ConeSensor.blue()+m_ConeSensor.red() > 1000;
    }

}