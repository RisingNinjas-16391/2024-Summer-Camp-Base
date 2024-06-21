package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakeSubsystem m_outtakeSubsystem;

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;
    private final GamepadButton m_outtake;

    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_intakeSubsystem = new IntakeSubsystem(hwMap, "intake");
        m_outtakeSubsystem = new IntakeSubsystem(hwMap, "outtake");

        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        m_outtake = new GamepadButton(m_driverController, GamepadKeys.Button.A);

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

        m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        m_outtakeSubsystem.setDefaultCommand(new IntakeCommand(m_outtakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
    }

    public void configureButtonBindings() {
        m_outtake.whileHeld(new IntakeCommand(m_intakeSubsystem, () -> -0.5));
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_intakeSubsystem, m_outtakeSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_intakeSubsystem, m_outtakeSubsystem).schedule();
                break;
        }

    }
}