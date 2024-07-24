package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Claw2Command;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.PivotPowerCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final SingleMotorSubsystem m_intakeSubsystem;
    private final SingleMotorSubsystem m_shooterSubsystem;
    private final ClawSubsystem m_clawSubsystem;
    private final Claw2Subsystem m_claw2Subsystem;
    private final WristSubsystem m_wristSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    private final GamepadEx m_driverController;

    private final GamepadButton m_resetHeading;
    //private final GamepadButton m_autoscore;
    //private final GamepadButton m_reverseIntake;
    private final GamepadButton m_clawOpen;
    private final GamepadButton m_clawClose;
    private final GamepadButton m_flipScore;
    private final GamepadButton m_flipfeed;
    //private final GamepadButton m_up;
    //private final GamepadButton m_down;
    private final GamepadButton m_low;
    private final GamepadButton m_high;
    private final GamepadButton m_feed;


    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_intakeSubsystem = new SingleMotorSubsystem(hwMap, "intake");
        m_shooterSubsystem = new SingleMotorSubsystem(hwMap, "shooter");
        m_clawSubsystem = new ClawSubsystem(hwMap);
        m_claw2Subsystem = new Claw2Subsystem(hwMap);
        m_wristSubsystem = new WristSubsystem(hwMap);
        m_pivotSubsystem = new PivotSubsystem(hwMap);

        m_driverController = new GamepadEx(gamepad1);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);
        //m_autoscore = new GamepadButton(m_driverController,GamepadKeys.Button.A);
        //m_reverseIntake = new GamepadButton(m_driverController, GamepadKeys.Button.);
        m_clawOpen = new GamepadButton(m_driverController, GamepadKeys.Button.X);
        m_clawClose = new GamepadButton(m_driverController, GamepadKeys.Button.B);
        m_flipScore = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_RIGHT);
        m_flipfeed = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_LEFT);

        //  m_up = new GamepadButton(m_driverController, GamepadKeys.Button.Y);
        // m_down = new GamepadButton(m_driverController, GamepadKeys.Button.A);

        m_low = new GamepadButton(m_driverController, GamepadKeys.Button.A);
        m_high = new GamepadButton(m_driverController, GamepadKeys.Button.Y);
        m_feed = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        m_driveSubsystem.updateTelemetry(telemetry);
        m_intakeSubsystem.updateTelemetry(telemetry);
        m_clawSubsystem.updateTelemetry(telemetry);
        m_claw2Subsystem.updateTelemetry(telemetry);
        telemetry.update();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));

        //m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        //m_shooterSubsystem.setDefaultCommand(new IntakeCommand(m_shooterSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

    }

    public void configureButtonBindings() {
        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));
        /*m_autoscore.whenPressed(new SequentialCommandGroup(
                new IntakeCommand(m_intakeSubsystem,-1).withTimeout(200),
                new IntakeCommand(m_shooterSubsystem,.7).withTimeout(200),
                new IntakeCommand(m_intakeSubsystem,1).withTimeout(1200).alongWith(
                        new IntakeCommand(m_shooterSubsystem,.7).withTimeout(1000))
        ));*/
        //m_reverseIntake.whenHeld(new IntakeCommand(m_intakeSubsystem,-1));

        //m_up.whenHeld(new ParallelCommandGroup(
                //new WristCommand(m_wristSubsystem,120),
                //new IntakeCommand(m_intakeSubsystem,.5)
               // ));
        //m_down.whenHeld(new ParallelCommandGroup(
               // new WristCommand(m_wristSubsystem,-30),
               // new IntakeCommand(m_intakeSubsystem,-.3)
               // ));

        m_clawOpen.whenPressed(new ParallelCommandGroup(
                new ClawCommand(m_clawSubsystem, 0),
                new Claw2Command(m_claw2Subsystem, 0)
        ));
        m_clawClose.whenPressed(new ParallelCommandGroup(
                new ClawCommand(m_clawSubsystem, 100),
                new Claw2Command(m_claw2Subsystem, -40)
        ));

        m_feed.whenPressed(new SequentialCommandGroup(
                new WristCommand(m_wristSubsystem, 20).withTimeout(500),
                new PivotCommand(m_pivotSubsystem, Math.toRadians(-65))
        ));
        m_low.whenPressed(new ParallelCommandGroup(
                new WristCommand(m_wristSubsystem, 160),
                new PivotCommand(m_pivotSubsystem, Math.toRadians(-16))
                        ));
        m_high.whenPressed(new ParallelCommandGroup(
                new WristCommand(m_wristSubsystem,160),
                new PivotCommand(m_pivotSubsystem, Math.toRadians(-21))
                ));
       // m_flipScore.whenPressed(new WristCommand(m_wristSubsystem, 120));
       // m_flipfeed.whenPressed(new WristCommand(m_wristSubsystem,-30));

        //m_up.whenHeld(new IntakeCommand(m_intakeSubsystem, .5));
        //m_down.whenHeld(new IntakeCommand(m_intakeSubsystem, -.5));

       //m_low.whenPressed(new IntakeCommand(m_intakeSubsystem, .5).withTimeout(3000));
        //m_high.whenPressed(new IntakeCommand(m_intakeSubsystem, .5).withTimeout(2500));

    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_wristSubsystem, m_pivotSubsystem, m_clawSubsystem, m_claw2Subsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem,  m_wristSubsystem, m_pivotSubsystem, m_clawSubsystem, m_claw2Subsystem).schedule();
                break;
        }

    }
}