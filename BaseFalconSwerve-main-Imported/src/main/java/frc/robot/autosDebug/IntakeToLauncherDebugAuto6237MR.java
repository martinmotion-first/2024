package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        Command intakeCommand = new RunIntakeCommand6237MR(intake);
        Command backIntoNote = new MoveByMetersCommand6237MR(s_Swerve, -1, 0);
        Command armToScoring = new ArmToScoringPostionCommand6237MR(arm);
        Command resetMovementPositionToStart = new MoveByMetersCommand6237MR(s_Swerve, 0, 1);
        // Command waitForPositioning = new WaitCommand(Constants.Arm.kAutonomousArmWaitTime);
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);

        addRequirements(intake, launcher);

        addCommands(
            armToFloor,
            intakeCommand,
            backIntoNote,
            armToScoring,
            resetMovementPositionToStart,
            // waitForPositioning,
            fireLauncherCommand
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
