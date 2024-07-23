package org.firstinspires.ftc.teamcode.subsystems.wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSubsystem extends SubsystemBase {
    private final ColorSensor m_ConeSensor;

    public ColorSubsystem(HardwareMap hwMap){
        m_ConeSensor = hwMap.get(ColorSensor.class, "color");
    }

    public void LEDon(boolean on){
        m_ConeSensor.enableLed(on);
    }

    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("Color", m_ConeSensor.blue() + m_ConeSensor.red());
    }
    public boolean hasCone(){
        return m_ConeSensor.blue() + m_ConeSensor.red() > 500;
    }
}
