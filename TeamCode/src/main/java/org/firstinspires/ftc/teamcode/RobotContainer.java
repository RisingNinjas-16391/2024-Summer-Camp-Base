package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.IndexerCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterPowerCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;

    private final GamepadEx m_driverController;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IndexerSubsystem m_indexerSubsystem;

    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_shooterSubsystem = new ShooterSubsystem(hwMap);
        m_indexerSubsystem = new IndexerSubsystem(hwMap);

        m_driverController = new GamepadEx(gamepad1);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        m_driveSubsystem.updateTelemetry(telemetry);
        m_shooterSubsystem.updateTelemetry(telemetry);

        telemetry.update();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));
        m_shooterSubsystem.setDefaultCommand(new ShooterPowerCommand(m_shooterSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        m_indexerSubsystem.setDefaultCommand(new IndexerCommand(m_indexerSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
    }

    public void configureButtonBindings() {
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_shooterSubsystem).schedule();
                break;
        }

    }
}