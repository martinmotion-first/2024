package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.RunLauncherCommand6237MR;
import frc.robot.commands.StopRunningLauncher6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class IntakeToLauncherDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    public IntakeToLauncherDebugAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        Command armToFloor = new ArmToIntakePositionCommand6237MR(arm);
        Command intakeOn = intake.feedLauncher(launcher);
        Command pauseOne = new WaitCommand(1);
        Command armToScoring = new ArmToScoringPostionCommand6237MR(arm);
        Command fireLauncherCommand = new RunLauncherCommand6237MR(launcher);
        Command pauseCommand = new WaitCommand(1);
        Command stopLauncherCommand = new StopRunningLauncher6237MR(launcher);
        addRequirements(intake, launcher);

        addCommands(
            armToFloor,
            intakeOn,
            pauseOne,
            armToScoring,
            fireLauncherCommand,
            pauseCommand,
            stopLauncherCommand
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
