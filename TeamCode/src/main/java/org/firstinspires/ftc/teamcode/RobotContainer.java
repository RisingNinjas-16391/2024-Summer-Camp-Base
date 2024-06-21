package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.PivotPowerCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;

    private final IntakeSubsystem m_intakeSubsystem;

    private final ClawSubsystem m_clawSubsystem;
    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    private final GamepadButton m_clawUp;
    private final GamepadButton m_clawDown;
    private final GamepadButton m_feed;
    private final GamepadButton m_autoBackFeed;
    private final GamepadButton m_autoFrontFeed;

    private final PivotSubsystem m_pivotSubsystem;
    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_intakeSubsystem = new IntakeSubsystem(hwMap);
        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);
        m_clawSubsystem = new ClawSubsystem(hwMap);
        m_pivotSubsystem = new PivotSubsystem(hwMap);

        m_clawUp = new GamepadButton(m_driverController, GamepadKeys.Button.RIGHT_BUMPER);
        m_clawDown = new GamepadButton(m_driverController, GamepadKeys.Button.LEFT_BUMPER);
        m_feed = new GamepadButton(m_driverController, GamepadKeys.Button.A);
        m_autoBackFeed = new GamepadButton(m_driverController, GamepadKeys.Button.Y);
        m_autoFrontFeed = new GamepadButton(m_driverController, GamepadKeys.Button.X);

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
        m_clawSubsystem.updateTelemetry(telemetry);

        telemetry.update();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));
//
//        m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> (
//                m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) -
//                m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
//                )));

        m_pivotSubsystem.setDefaultCommand(new PivotPowerCommand(
                m_pivotSubsystem, () -> (m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))));
    }

    public void configureButtonBindings() {
        //Claw
        m_clawUp.whenPressed(new ClawCommand(m_clawSubsystem, 0));
        m_clawDown.whenPressed(new ClawCommand(m_clawSubsystem, 90));

        m_feed.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(-15)));
        m_autoFrontFeed.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(40)));

        m_autoBackFeed.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(180)));
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_pivotSubsystem, m_clawSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_pivotSubsystem, m_clawSubsystem).schedule();
                break;
        }

    }
}