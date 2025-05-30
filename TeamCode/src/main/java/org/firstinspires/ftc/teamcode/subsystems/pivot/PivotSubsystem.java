package org.firstinspires.ftc.teamcode.subsystems.pivot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;

    //TODO: Tune kP for arm. If the arm moves too fast lower, if it moves too slow increase
    public static PIDFController kPIDF = new PIDFController(2,0,0,0.2);

    //TODO: Replace with preferred starting angle upon initialization
    private double desiredAngle = Math.toRadians(90);

    //TODO: Tune for arm, if the arm goes up without doing anything lower, if it falls then increase it
    public static double kG = 0.3;

    //TODO: Replace with starting angle offset
    public static double angleOffset = 110;

    public static double tolerance = 0.2;

    public PivotSubsystem(@NonNull HardwareMap hwMap){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kPIDF.setTolerance(tolerance);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Pivot")
                .addData("\nEncoder Ticks Pivot:", pivot.getCurrentPosition())
                .addData("\nPivot Angle Degrees", Math.toDegrees(getAngle()))
                .addData("\nDesired Pivot Angle Degrees", Math.toDegrees(desiredAngle))
                .addData("\nPivot Power", calculatePID())
                .addData("\nAt Setpoint", atSetpoint());

        telemetry.update();
    }

    public void setPower(double power){
        pivot.setPower(power);
    }

    public double getAngle() {
        return pivot.getCurrentPosition() * ((22 * 2 * Math.PI) / (28 * 100 * 66)) + Math.toRadians(angleOffset);
    }

    public void setAngle(double angle){
        desiredAngle = angle;
    }

    public double calculatePID() {
        return kPIDF.calculate(getAngle(), desiredAngle) - Math.sin(getAngle()) * kG;
    }

    public boolean atSetpoint() {
        return (Math.abs(desiredAngle - getAngle()) < 0.4);
    }

    public boolean isBusy() {
        return pivot.isBusy();
    }

    @Override
    public void periodic() {
        setPower(calculatePID());

    }



}