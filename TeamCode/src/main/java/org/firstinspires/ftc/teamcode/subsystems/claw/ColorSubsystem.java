package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ColorSubsystem extends SubsystemBase {
    private final ColorSensor m_ConeSensor;

    public ColorSubsystem(HardwareMap hwMap) {
        m_ConeSensor = new ColorSensor() {
            @Override
            public int red() {
                return 0;
            }

            @Override
            public int green() {
                return 0;
            }

            @Override
            public int blue() {
                return 0;
            }

            @Override
            public int alpha() {
                return 0;
            }

            @Override
            public int argb() {
                return 0;
            }

            @Override
            public void enableLed(boolean b) {

            }

            @Override
            public void setI2cAddress(I2cAddr i2cAddr) {

            }

            @Override
            public I2cAddr getI2cAddress() {
                return null;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return null;
            }

            @Override
            public String getConnectionInfo() {
                return null;
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("red Value",m_ConeSensor.red());

        telemetry.addData("blue Value",m_ConeSensor.blue());

        telemetry.addData("green Value",m_ConeSensor.green());
    }

    public void turnToAngle(double angle) {
        //m_claw.turnToAngle(angle);

    }

}