package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive m_drive;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Boolean fieldCentric) {
        m_drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    }

}
