package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class OperatorMapping6237MR {
    public static void mapXboxController(XboxController operatorController, IntakeSubsystem intake, ArmSubsystem arm, LauncherSubsystem launcher){
        //revbot subsytem configurations
        // button to put swerve modules in an "x" configuration to hold position
    // new JoystickButton(operator, XboxController.Button.kLeftStick.value)
    //     .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // set up arm preset positions
    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    new Trigger(
            () ->
                operatorController.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kIntakePosition)));

    new JoystickButton(operatorController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kHomePosition)));

    // intake controls (run while button is held down, run retract command once when the button is released)
    new Trigger(
            () ->
                operatorController.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> intake.setPower(Constants.Intake.kIntakePower), intake))
        .onFalse(intake.retract());

    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> intake.setPower(-1.0)));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> launcher.runLauncher(), launcher));

    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .onTrue(intake.feedLauncher(launcher));
    }
}
