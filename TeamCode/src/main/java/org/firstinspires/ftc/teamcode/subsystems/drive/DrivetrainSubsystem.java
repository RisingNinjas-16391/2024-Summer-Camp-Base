package org.firstinspires.ftc.teamcode.subsystems.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drive.trajectorysequence.TrajectorySequenceBuilder;

@Config
public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive m_drive;
    private final PIDFController kHeadingPID;
    private double desiredHeading = 0;

    public static double omegaSpeed = 0.5;
    private final boolean m_fieldCentric;



    public DrivetrainSubsystem(@NonNull HardwareMap hardwareMap, Boolean fieldCentric) {
        m_drive = new MecanumDrive(hardwareMap);
        kHeadingPID = new PIDFController(DriveConstants.TELEOP_HEADING_PID);
        kHeadingPID.setInputBounds(0, 2 * Math.PI);

        m_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_fieldCentric = fieldCentric;
    }

    @Override
    public void periodic() {
        try {
            m_drive.updateJustPoseEstimate();

        } catch (Exception e) {

        }


    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Drivetrain");
        telemetry.addData("Heading", m_drive.getPoseEstimate().getHeading());
        telemetry.addData("Desired Heading", desiredHeading);
    }

    public void setDrivePower(@NonNull Pose2d drivePower) {
        Pose2d poseEstimate = m_drive.getPoseEstimate();

        Vector2d input = new Vector2d(drivePower.getY(), drivePower.getX()).rotated(
                m_fieldCentric ? -poseEstimate.getHeading() : 0
        );

        m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        drivePower.getHeading()
                )
        );

    }

    public void setDrivePowerHeadingPID(@NonNull Pose2d drivePower) {
        updateHeadingPID(drivePower);
    }

    public void runTrajectory(TrajectorySequence trajectory) {
        m_drive.followTrajectorySequenceAsync(trajectory);
    }

    public void update() {
        m_drive.update();
    }

    public void updateHeadingPID(Pose2d drivePower) {
        setDrivePower(new Pose2d(drivePower.getX(), drivePower.getY(), calculatePID()));

        if (Math.abs(drivePower.getHeading()) > 0.1) {
            setHeading((getHeading() + (drivePower.getHeading()) * omegaSpeed));
        }
    }

    public void setHeading(double heading) {
        desiredHeading = heading;
    }

    public double calculatePID() {
        kHeadingPID.setTargetPosition(desiredHeading);
        return kHeadingPID.update(getHeading());
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return m_drive.trajectorySequenceBuilder(startPose);
    }

    public double getHeading() {
        return m_drive.getRawExternalHeading();
    }

    public Pose2d getPoseEstimate() {
        return m_drive.getPoseEstimate();
    }

    public boolean isBusy() {
        return m_drive.isBusy();
    }

    public boolean isFinishedHeadingPID() {
        return (desiredHeading - getHeading()) < 0.2;
    }
}