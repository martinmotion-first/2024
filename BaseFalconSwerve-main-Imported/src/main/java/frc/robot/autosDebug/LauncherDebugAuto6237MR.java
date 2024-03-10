package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.RunLauncherCommand6237MR;
import frc.robot.commands.StopRunningLauncher6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LauncherDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
 
        public LauncherDebugAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        Command fireLauncherCommand = new RunLauncherCommand6237MR(launcher);
        Command pauseCommand = new WaitCommand(1);
        Command stopLauncherCommand = new StopRunningLauncher6237MR(launcher);
        addRequirements(launcher);
        addCommands(
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
