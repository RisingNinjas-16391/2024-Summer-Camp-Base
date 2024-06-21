package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;

import java.util.function.DoubleSupplier;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;

    private final IntakeSubsystem m_intakeSubsystem;

    private final OuttakeSubsystem m_outtakeSubsystem;

    private final GamepadEx m_driverController;

    private final GamepadButton m_rintakePosition;

    private final GamepadButton m_routtakePosition;


    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_intakeSubsystem = new IntakeSubsystem(hwMap);
        m_outtakeSubsystem = new OuttakeSubsystem(hwMap);
        m_driverController = new GamepadEx(gamepad1);


        m_rintakePosition = new GamepadButton(m_driverController, GamepadKeys.Button.RIGHT_BUMPER);
        m_routtakePosition = new GamepadButton(m_driverController, GamepadKeys.Button.LEFT_BUMPER);


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
        m_intakeSubsystem.setDefaultCommand(new IntakeCommand(
                m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        m_outtakeSubsystem.setDefaultCommand(new OuttakeCommand(
                m_outtakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
    }

    public void configureButtonBindings() {
        m_rintakePosition.whileActiveOnce(new IntakeCommand(m_intakeSubsystem, () -> -312.0));
        m_routtakePosition.whileActiveOnce(new OuttakeCommand(m_outtakeSubsystem, () -> -312.0));
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