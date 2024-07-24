package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    //private final SingleMotorSubsystem m_intakeSubsystem;
    //private final SingleMotorSubsystem m_shooterSubsystem;
    private final WristSubsystem m_wristSubsystem;
    private final ClawSubsystem m_clawSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    private final GamepadEx m_driverController;

    private final GamepadButton m_resetHeading;

    private final GamepadButton m_autoscore;

    private final GamepadButton m_reverseIntake;
    private final GamepadButton m_stow;
    private final GamepadButton m_feed;


    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        //m_intakeSubsystem = new SingleMotorSubsystem(hwMap, "intake");
        //m_shooterSubsystem = new SingleMotorSubsystem(hwMap, "shooter");
        m_wristSubsystem = new WristSubsystem(hwMap);
        m_clawSubsystem = new ClawSubsystem(hwMap);
        m_pivotSubsystem = new PivotSubsystem(hwMap);

        m_driverController = new GamepadEx(gamepad1);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        m_autoscore = new GamepadButton(m_driverController, GamepadKeys.Button.A);
        m_stow = new GamepadButton(m_driverController,GamepadKeys.Button.X);
        m_feed = new GamepadButton(m_driverController, GamepadKeys.Button.Y);

        m_reverseIntake = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN);

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

        //m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        //m_shooterSubsystem.setDefaultCommand(new IntakeCommand(m_shooterSubsystem, () -> -.7*m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
    }

    public void configureButtonBindings() {
        m_stow.whenPressed(new SequentialCommandGroup(
                new ClawCommand(m_clawSubsystem,90),
                new WristCommand(m_wristSubsystem,90),
                new PivotCommand(m_pivotSubsystem,90)
        ));
        m_feed.whenPressed(new SequentialCommandGroup(
                new PivotCommand(m_pivotSubsystem,30),
                new WristCommand(m_wristSubsystem, 30),
                new ClawCommand(m_clawSubsystem, 90)
        ));
        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));
        /*
        m_autoscore.whenPressed(new SequentialCommandGroup(
                new IntakeCommand(m_intakeSubsystem, -.4).withTimeout(500),
                new IntakeCommand(m_shooterSubsystem, .8).withTimeout(3000),
                new IntakeCommand(m_intakeSubsystem, 1).withTimeout(500)
        ));
        m_reverseIntake.whenHeld(new IntakeCommand(m_intakeSubsystem, -1));*/
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_wristSubsystem, m_clawSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_wristSubsystem, m_clawSubsystem).schedule();
                break;
        }

    }
}