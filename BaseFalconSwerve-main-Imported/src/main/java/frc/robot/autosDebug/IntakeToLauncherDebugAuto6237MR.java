package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.FireLauncherCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.RunIntakeCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeToLauncherDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    public IntakeToLauncherDebugAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        Command armToFloor = new ArmToIntakePositionCommand6237MR(arm);
        Command waitForPositioningArmPositioning1 = new WaitCommand(Constants.AutonomousModeConstants.kAutonomousArmWaitTime);
        Command intakeCommand = new RunIntakeCommand6237MR(intake);
        Command backIntoNote = MoveByMetersCommand6237MR.Create(s_Swerve, -1, 0);
        Command armToScoring = new ArmToScoringPostionCommand6237MR(arm);
        Command resetMovementPositionToStart = MoveByMetersCommand6237MR.Create(s_Swerve, 1, 0);
        Command waitForPositioningArmPositioning2 = new WaitCommand(Constants.AutonomousModeConstants.kAutonomousArmWaitTime);
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
        
        Command deleteMeYPlusOne = MoveByMetersCommand6237MR.Create(s_Swerve, 0, 1);
        Command deleteMeYMinusOne = MoveByMetersCommand6237MR.Create(s_Swerve, 0, -1);
        // Command additionalCommand = MoveByMetersCommand6237MR.Create(s_Swerve,1, 0);

        addRequirements(intake, launcher);

        addCommands(
            deleteMeYPlusOne,
            armToFloor,
            waitForPositioningArmPositioning1,
            intakeCommand,
            backIntoNote,
            armToScoring,
            resetMovementPositionToStart,
            waitForPositioningArmPositioning2,
            fireLauncherCommand,
            deleteMeYMinusOne
            // additionalCommand
        );
    }

        @Override
        public double getSimulatorDisplayCoordinateX() {
            return 0;
        }

        @Override
        public double getSimulatorDisplayCoordinateY() {
            return 0;
        }
}
