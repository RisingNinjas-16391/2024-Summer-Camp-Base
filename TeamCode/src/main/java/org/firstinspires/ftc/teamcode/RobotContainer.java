
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final SingleMotorSubsystem m_intakeSubsystem;
    private final SingleMotorSubsystem m_shooterSubsystem;
    private final PivotSubsystem m_pivotSubsystem;
    private final WristSubsystem m_wristSubsystem;
    private final ClawSubsystem m_claw;
    private final Claw2Subsystem m_claw2;
    private final ColorSubsystem m_ConeSensor;

    private final GamepadEx m_driverController;
    private final GamepadButton m_clawOpen;
    private final GamepadButton m_clawClose;
//    private final GamepadButton m_up;
//    private final GamepadButton m_down;
    private final GamepadButton m_feed;
    private final GamepadButton m_low;
    private final GamepadButton m_high;
    private final GamepadButton m_score;
    private final GamepadButton m_resetHeading;
    private final GamepadButton m_autoFeed;
    private final GamepadButton m_wristFront;
    private final GamepadButton m_wristBack;
    private final Trigger m_hasCone;


    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_intakeSubsystem = new SingleMotorSubsystem(hwMap, "intake");
        m_shooterSubsystem = new SingleMotorSubsystem(hwMap, "shooter");
        m_pivotSubsystem = new PivotSubsystem(hwMap);
        m_wristSubsystem = new WristSubsystem(hwMap);
        m_claw = new ClawSubsystem(hwMap);
        m_claw2 = new Claw2Subsystem(hwMap);
        m_ConeSensor = new ColorSubsystem(hwMap);
        m_hasCone = new Trigger(m_ConeSensor::hasCone);

        m_driverController = new GamepadEx(gamepad1);

        m_clawOpen = new GamepadButton(m_driverController, GamepadKeys.Button.LEFT_BUMPER);
        m_clawClose = new GamepadButton(m_driverController, GamepadKeys.Button.RIGHT_BUMPER);
        m_feed = new GamepadButton(m_driverController,GamepadKeys.Button.A);
        m_low = new GamepadButton(m_driverController, GamepadKeys.Button.X);
        m_high = new GamepadButton(m_driverController, GamepadKeys.Button.Y);

        m_score = new GamepadButton(m_driverController,GamepadKeys.Button.A);

        m_wristFront = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_LEFT);
        m_wristBack = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_RIGHT);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);
        m_autoFeed = new GamepadButton(m_driverController,GamepadKeys.Button.B);


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
        m_wristSubsystem.updateTelemetry(telemetry);
        m_ConeSensor.updateTelemetry(telemetry);
        telemetry.addData("HasCone?", m_hasCone);

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

        m_clawOpen.whenPressed(new ParallelCommandGroup(
                new ClawCommand(m_claw, 5),
                new Claw2Command(m_claw2, 35)
        ));
        m_clawClose.whenPressed(new ParallelCommandGroup(
                new ClawCommand(m_claw, 45),
                new Claw2Command(m_claw2, 15)
        ));

        m_autoFeed.whenPressed(new SequentialCommandGroup(
                new WristCommand(m_wristSubsystem, 2).withTimeout(500),
                new PivotCommand(m_pivotSubsystem, Math.toRadians(0)),
                new WaitUntilCommand(m_ConeSensor::hasCone),
                new ParallelCommandGroup(
                        new ClawCommand(m_claw, 45),
                        new Claw2Command(m_claw2, 0)).withTimeout(500)
                ));

        m_low.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(58)),
                new WristCommand(m_wristSubsystem, 125)));

        m_high.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(50)),
                new WristCommand(m_wristSubsystem, 125)));

        m_score.whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(new PivotCommand(m_pivotSubsystem, Math.toRadians(55)),
                        new ParallelCommandGroup(
                                new ClawCommand(m_claw, 5),
                                new Claw2Command(m_claw2, 35)
                        )),
                new SequentialCommandGroup(new PivotCommand(m_pivotSubsystem, Math.toRadians(65)),
                        new ParallelCommandGroup(
                                new ClawCommand(m_claw, 5),
                                new Claw2Command(m_claw2, 35))),
                (() -> m_pivotSubsystem.getAngle() == 50)));

       /* m_feed.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(6)),
                new WristCommand(m_wristSubsystem, 0)));*/




    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_pivotSubsystem, m_claw, m_claw2, m_wristSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_pivotSubsystem, m_claw, m_claw2, m_wristSubsystem).schedule();
                break;
        }

    }
}