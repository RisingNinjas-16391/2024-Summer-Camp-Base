package org.firstinspires.ftc.teamcode.subsystems.shooter;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooter;
    private static final double gearRatio = 1;
    private final PIDFController kShooterController = new PIDFController(0.08, 0, 0, 0);
    private double desiredRPM = 0;
    private double desiredPower = 0;

    public ShooterSubsystem(@NonNull HardwareMap hwMap){
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Current RPM:", getRPM());
        telemetry.addData("Desired RPM:", desiredRPM);
        telemetry.addData("Power", desiredPower);
        telemetry.addData("Is Finished: ", shooter.isBusy());
        telemetry.addData("Amperage", shooter.getCurrent(CurrentUnit.AMPS));
    }
    @Override
    public void periodic() {
        try {
            setPower(desiredPower);

        } catch (Exception e) {

        }
    }

    public void setRPM(double RPM) {
        desiredRPM = RPM;
    }

    public double getRPM() {
        return shooter.getVelocity(AngleUnit.RADIANS) * (60.0 / (2 * Math.PI)) * gearRatio;
    }

    public void calculatePID() {
        desiredPower = kShooterController.calculate(getRPM(), desiredRPM);
    }
    public void setPower(double power){
        desiredPower = power;
        shooter.setPower(desiredPower);
    }

    public double getPower(){
        return shooter.getPower();
    }

    public boolean isBusy() {
        return shooter.isBusy();
    }

    public boolean atSetpoint() {
        if (Math.abs(desiredRPM - getRPM()) < 10) {
            return true;
        } else {
            return false;
        }
    }
}