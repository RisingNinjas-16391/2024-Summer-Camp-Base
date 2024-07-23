package org.firstinspires.ftc.teamcode.subsystems.wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*public class WristSubsystem {package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;*/

    public class WristSubsystem extends SubsystemBase {
        private final SimpleServo m_wrist;

        public WristSubsystem(HardwareMap hwMap) {
            m_wrist = new SimpleServo(hwMap, "wrist", -100,
                    180, AngleUnit.DEGREES);
            turnToAngle(-7);
        }

        public void updateTelemetry(Telemetry telemetry) {
            telemetry.addLine("Wrist");
            telemetry.addData("Angle", m_wrist.getAngle());
        }

        public void turnToAngle(double angle) {
            m_wrist.turnToAngle(angle);

        }


}
