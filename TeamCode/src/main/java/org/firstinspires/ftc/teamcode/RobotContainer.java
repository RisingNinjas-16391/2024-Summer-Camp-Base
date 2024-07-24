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
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
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
    //private final SingleMotorSubsystem m_pivot;
    private final PivotSubsystem m_pivotSubsystem;

    private final ClawSubsystem m_claw;
    private final WristSubsystem m_wrist;

    private final GamepadEx m_driverController;
    private final GamepadButton m_clawOpen;
    private final GamepadButton m_clawClose;
    private final GamepadButton m_reset;
    private final GamepadButton m_lowScore;
    private final GamepadButton m_highScore;

    private final GamepadButton m_resetHeading;






    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        //m_intakeSubsystem = new SingleMotorSubsystem(hwMap, "intake");
        //m_pivot = new SingleMotorSubsystem(hwMap);

        m_pivotSubsystem = new PivotSubsystem(hwMap);

        m_claw = new ClawSubsystem(hwMap, "claw");
        m_wrist = new WristSubsystem(hwMap, "wrist");

        m_driverController = new GamepadEx(gamepad1);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        m_clawOpen = new GamepadButton(m_driverController,GamepadKeys.Button.X);
        m_clawClose = new GamepadButton(m_driverController,GamepadKeys.Button.Y);

        m_reset = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN);
        m_lowScore = new GamepadButton(m_driverController, GamepadKeys.Button.A);
        m_highScore = new GamepadButton(m_driverController, GamepadKeys.Button.B);


        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        m_driveSubsystem.updateTelemetry(telemetry);
        m_claw.updateTelemetry(telemetry);
        m_wrist.updateTelemetry(telemetry);
        m_pivotSubsystem.updateTelemetry(telemetry);
     //   if (toggle1 && toggle2==true) {

     //   }
            ;

        telemetry.update();
    }

    public void setDefaultCommands() {
        //m_highScore.whenHeld(new IntakeCommand(m_pivot,.7));
        //m_lowScore.whenHeld(new IntakeCommand(m_pivot,-.5));
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));

        //m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
    }

    public void configureButtonBindings() {
        m_highScore.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem,Math.toRadians(-41)),
                new WristCommand(m_wrist, 120)
        ));
        m_lowScore.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(-44)),
                new WristCommand(m_wrist, 120)
        ));
        m_reset.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(0)),
                new WristCommand(m_wrist, 0),
                new ClawCommand(m_claw, -20)

        ));


        //m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));

        m_clawOpen.whenPressed(
                new ClawCommand(m_claw, -20)
        );
        m_clawClose.whenPressed(
                new ClawCommand(m_claw, 25)
              );
        /*m_intake.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(0)),
                new WristCommand(m_wrist, 0)
        ));
        m_lowScore.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(40)),
                new WristCommand(m_wrist, 45)
        ));
        m_highScore.whenPressed(new ParallelCommandGroup(
                new PivotCommand(m_pivotSubsystem, Math.toRadians(55)),
                new WristCommand(m_wrist, 55)
        ));*/

      //  m_outtakeB.toggleWhenPressed(
      //          new ClawCommand(m_clawSubsystem,35).withTimeout(100),
       //         new IntakeCommand(m_shooterSubsystem,1)
      //  );

        //m_outtakeA.whenPressed(new SequentialCommandGroup(
              // new ClawCommand(m_clawSubsystem,0).withTimeout(300),
              //  new IntakeCommand(m_shooterSubsystem,.55).withTimeout(4000)
      //  ));
    }


    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_pivotSubsystem, m_claw, m_wrist).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_pivotSubsystem, m_claw, m_wrist).schedule();
                break;
        }

    }
}