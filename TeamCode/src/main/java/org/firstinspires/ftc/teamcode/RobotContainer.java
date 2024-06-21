package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;

    private final IntakeSubsystem m_intakeSubsystem;

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_intakeSubsystem = new IntakeSubsystem(hwMap);

        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        m_driveSubsystem.updateTelemetry(telemetry);

        telemetry.update();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));

        m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> (
                m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) -
                m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                )));
    }

    public void configureButtonBindings() {
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new RedAutoCommand(m_driveSubsystem, m_intakeSubsystem).schedule();
                break;
            case 2:
                new BlueAutoCommand(m_driveSubsystem, m_intakeSubsystem).schedule();
                break;
        }

    }
}