package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.PivotPowerCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final PivotSubsystem m_pivotSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    private final GamepadEx m_driverController;

    private final GamepadButton m_outtakePosition;
    private final GamepadButton m_intakePosition;
    private final GamepadButton m_resetHeading;

    private final GamepadButton m_pivotUp;
    private final GamepadButton m_pivotDown;


    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, true);
        m_pivotSubsystem = new PivotSubsystem(hwMap);
        m_intakeSubsystem = new IntakeSubsystem(hwMap);

        m_driverController = new GamepadEx(gamepad1);

        m_outtakePosition = new GamepadButton(m_driverController, GamepadKeys.Button.X);
        m_intakePosition = new GamepadButton(m_driverController, GamepadKeys.Button.A);

        m_pivotUp = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_UP);
        m_pivotDown = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        m_driveSubsystem.updateTelemetry(telemetry);
        m_pivotSubsystem.updateTelemetry(telemetry);

        telemetry.update();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));

//        m_pivotSubsystem.setDefaultCommand(new PivotPowerCommand(
//                m_pivotSubsystem, () -> (m_driverController.getButton(GamepadKeys.Button.X) ? 1 : m_driverController.getButton(GamepadKeys.Button.B) ? -1 : 0)));

        m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
    }

    public void configureButtonBindings() {
        m_outtakePosition.whenPressed(new PivotCommand(m_pivotSubsystem, 30));
        m_intakePosition.whenPressed(new PivotCommand(m_pivotSubsystem, 0));
        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));

//        m_pivotUp.whileHeld(new PivotPowerCommand(m_pivotSubsystem, () -> -0.5));
//        m_pivotUp.whenInactive(new PivotPowerCommand(m_pivotSubsystem, () -> 0));
//
//        m_pivotDown.whileHeld(new PivotPowerCommand(m_pivotSubsystem, () -> 0.5));
//        m_pivotDown.whenInactive(new PivotPowerCommand(m_pivotSubsystem, () -> 0));

    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_intakeSubsystem, m_pivotSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_intakeSubsystem, m_pivotSubsystem).schedule();
                break;
        }

    }
}