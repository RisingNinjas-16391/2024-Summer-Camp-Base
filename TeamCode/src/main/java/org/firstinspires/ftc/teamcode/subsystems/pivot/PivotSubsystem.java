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
    private final DcMotorEx follower;

    //TODO: Tune kP for arm. If the arm moves too fast lower, if it moves too slow increase
    public static PIDFController kPIDF = new PIDFController( .3,0,0,0);

    //TODO: Replace with preferred starting angle upon initialization
    private double desiredAngle = Math.toRadians(0);

    //TODO: Tune for arm, if the arm goes up without doing anything lower, if it falls then increase it
    public static double kG = 0;

    //TODO: Replace with starting angle offset
    public static double angleOffset = 0;

    public static double tolerance = 0.1;

    public PivotSubsystem(@NonNull HardwareMap hwMap){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        follower = hwMap.get(DcMotorEx.class,"follower");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);
        follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        follower.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kPIDF.setTolerance(tolerance);

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Pivot");
        telemetry.addData("\nEncoder Ticks Pivot:", pivot.getCurrentPosition());
        telemetry.addData("\nPivot Angle Degrees", Math.toDegrees(getAngle()));
        telemetry.addData("\nDesired Pivot Angle Degrees", Math.toDegrees(desiredAngle));
        telemetry.addData("\nPivot Power", calculatePID());
        telemetry.addData("\nAt Setpoint", atSetpoint());
    }

    public void setPower(double power){
        pivot.setPower(power);
        follower.setPower(power);
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
        setPower(12 * -calculatePID());

    }



}