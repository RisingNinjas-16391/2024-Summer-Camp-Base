package org.firstinspires.ftc.teamcode.subsystems.slides;

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
public class SlidesSubsystem extends SubsystemBase {

    private final DcMotorEx Right;
    private final DcMotorEx Left;

    public static PIDFController kPIDF = new PIDFController(.75,0.2,0,0);


    private double desiredExtension = (0);

    public static double extensionOffset = 0;

    public static double tolerance = .05;



    public SlidesSubsystem(@NonNull HardwareMap hwMap){
        Right = hwMap.get(DcMotorEx.class, "pivot");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setDirection(DcMotorSimple.Direction.FORWARD);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kPIDF.setTolerance(tolerance);

        Left = hwMap.get(DcMotorEx.class,"slave");
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left.setDirection(DcMotorSimple.Direction.FORWARD);
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Slides");
        //telemetry.addData("Encoder Ticks Pivot:", pivot.getCurrentPosition());
        //telemetry.addData("Slide Extension", ge);
        //telemetry.addData("Desired Pivot Angle Degrees", Math.toDegrees(desiredAngle));
        //telemetry.addData("Pivot Power", calculatePID());
        //telemetry.addData("At Setpoint", atSetpoint());

    }
    public void setPower(double power){
        Left.setPower(power);
        Right.setPower(power);
    }

    public void setExtension(double extension){desiredExtension = extension;};

    public double calculatePID() {
        return kPIDF.calculate(getExtension(), desiredExtension) - Math.cos(getExtension());
    }
    public double getExtension() {
        try {
            return ((Right.getCurrentPosition() / 537.7)*1.5) + (extensionOffset);
        } catch (Exception e) {
            return 0;
        }
    }


    public boolean atSetpoint() {
        return (Math.abs(desiredExtension - getExtension()) < 0.2);
    }
    @Override
    public void periodic() {
        setPower(calculatePID());

    }
}