package org.firstinspires.ftc.teamcode.subsystems.indexer;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IndexerSubsystem extends SubsystemBase {
    private final DcMotorEx indexer;
    private double power = 0;
    public IndexerSubsystem(HardwareMap hwMap) {
        indexer = hwMap.get(DcMotorEx.class, "indexer");
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Indexer");
        telemetry.addData("Power", power);
    }
    public void setPower(double setPower) {
        power = setPower;
        indexer.setPower(power);
    }
}
