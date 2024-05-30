package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotContainer {

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    private final GamepadButton m_faceForward;
    private final GamepadButton m_faceLeft;
    private final GamepadButton m_faceRight;
    private final GamepadButton m_faceBackward;

    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        m_faceForward = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_UP);
        m_faceLeft = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_LEFT);
        m_faceRight = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_RIGHT);
        m_faceBackward = new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void periodic(Telemetry telemetry) {
        telemetry.update();
    }

    public void setDefaultCommands(){
    }

    public void configureButtonBindings() {
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
            case 2:

        }

    }
}