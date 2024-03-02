package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class DriverMapping6237MR {

    /* Driver Buttons */
    public static JoystickButton zeroGyro;
    public static JoystickButton robotCentric;

    /* Drive Controls */
    public static int translationAxis = XboxController.Axis.kLeftY.value;
    public static int strafeAxis = XboxController.Axis.kLeftX.value;
    public static int rotationAxis = XboxController.Axis.kRightX.value;

    public static void mapXboxController(XboxController driverController, Swerve swerveDrive) {
        zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
        robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

        zeroGyro.onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
    }
    
}
