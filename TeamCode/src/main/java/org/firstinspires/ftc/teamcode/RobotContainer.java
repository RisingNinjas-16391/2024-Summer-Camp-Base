package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
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
//import org.firstinspires.ftc.teamcode.commands.PivotPowerCommand;
import org.firstinspires.ftc.teamcode.commands.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.RedAutoCommand;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final ColorSubsystem m_cubeSensor;
    private final SlidesSubsystem m_slides;

    private final PivotSubsystem m_pivotSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    private final ClawSubsystem m_claw;

    private final GamepadEx m_driverController;

    //private final GamepadButton m_outtakePosition;
    private final GamepadButton m_score;
    private final GamepadButton m_intakePosition;
    private final GamepadButton m_autoScore;
    private final GamepadButton m_autoScore2;
    private final GamepadButton m_highscore;

    private final GamepadButton m_resetHeading;

    public final Trigger m_hasCone;

    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, false);
        m_pivotSubsystem = new PivotSubsystem(hwMap);
        m_intakeSubsystem = new IntakeSubsystem(hwMap);
        m_cubeSensor = new ColorSubsystem(hwMap);
        m_claw = new ClawSubsystem(hwMap);
        m_slides = new SlidesSubsystem(hwMap);
        m_hasCone = new Trigger(m_cubeSensor::hasCube);


        m_driverController = new GamepadEx(gamepad1);

        //m_outtakePosition = new GamepadButton(m_driverController, GamepadKeys.Button.A);
        m_intakePosition = new GamepadButton(m_driverController, GamepadKeys.Button.B);
        m_autoScore = new GamepadButton(m_driverController, GamepadKeys.Button.Y);
        m_autoScore2 = new GamepadButton(m_driverController, GamepadKeys.Button.X);
        m_score = new GamepadButton(m_driverController,GamepadKeys.Button.A);
        m_highscore = new GamepadButton(m_driverController,GamepadKeys.Button.DPAD_UP);



        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        //m_driveSubsystem.updateTelemetry(telemetry);
        m_cubeSensor.updateTelemetry(telemetry);
        //m_intakeSubsystem.updateTelemetry(telemetry);
        m_pivotSubsystem.updateTelemetry(telemetry);
        m_claw.updateTelemetry(telemetry);
        m_cubeSensor.LEDon(true);
        telemetry.addData("HasCone?",m_hasCone);


        telemetry.update();
    }


    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));

        m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-.2 ));
    }

    public void configureButtonBindings() {
m_intakePosition.whenPressed(new SequentialCommandGroup(
        new PivotCommand(m_pivotSubsystem,Math.toRadians(0)),
        new ExtensionCommand(m_slides,0),
        new WaitUntilCommand(m_cubeSensor::hasCube),
        new PivotCommand(m_pivotSubsystem,Math.toRadians(180)),
        new ExtensionCommand(m_slides,20)
))


        //m_outtakePosition.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(140)));
        //m_intakePosition.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(0)));
        //m_intakePosition.whenPressed(new ClawCommand(m_claw,0));
        //m_score.whenPressed(new ClawCommand(m_claw,45));
        //m_autoScore.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(60)));
        //m_autoScore2.whenPressed(new PivotCommand(m_pivotSubsystem,Math.toRadians(110)));

        //m_outtakePosition.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(200)));
        //m_intakePosition.whenPressed(new PivotCommand(m_pivotSubsystem, Math.toRadians(0)));
        /*m_intakePosition.whenPressed(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PivotCommand(m_pivotSubsystem, Math.toRadians(0)).withTimeout(500),
                        new ClawCommand(m_claw,0),
                ).withTimeout(500),
                new WaitUntilCommand(m_ConeSensor::hasCone),
                new ClawCommand(m_claw,45).withTimeout(150),
                new PivotCommand(m_pivotSubsystem,Math.toRadians(30))
        ));




      m_autoScore.whenPressed(new SequentialCommandGroup(
              new ParallelCommandGroup(
                      new PivotCommand(m_pivotSubsystem,Math.toRadians(150)),
                      new WristCommand(m_wrist,180)
              ).withTimeout(500),
                new WaitUntilCommand(m_score::get),
                new PivotCommand(m_pivotSubsystem,Math.toRadians(160)).withTimeout(100),
                new ClawCommand(m_claw,0).withTimeout(200),
              new ParallelCommandGroup(
                      new PivotCommand(m_pivotSubsystem,Math.toRadians(30)),
                      new WristCommand(m_wrist,0)
              ).withTimeout(500)
                ));
      m_highscore.whenPressed(new PivotCommand(m_pivotSubsystem,Math.toRadians(140)));


        m_autoScore2.whenPressed(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PivotCommand(m_pivotSubsystem,Math.toRadians(173)),
                        new WristCommand(m_wrist,180)
                ).withTimeout(500),
                new WaitUntilCommand(m_score::get),
                new PivotCommand(m_pivotSubsystem,Math.toRadians(183)).withTimeout(100),
                new ClawCommand(m_claw,0).withTimeout(200),
                new ParallelCommandGroup(
                        new PivotCommand(m_pivotSubsystem,Math.toRadians(30)),
                        new WristCommand(m_wrist,0)
                ).withTimeout(500)*/;




        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));

        //m_hasCone.whenActive()
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem, m_intakeSubsystem, m_pivotSubsystem,m_claw).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_claw, m_cubeSensor).schedule();
                break;
        }

    }
}