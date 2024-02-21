package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.PauseCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class IntakeDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
 
    public IntakeDebugAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        Command turnIntakeOn = intake.feedLauncher(launcher);
        Command pauseCommand = new PauseCommand6237MR(2000);
        Command intakeOff = intake.retract();

        addCommands(
            turnIntakeOn,
            pauseCommand,
            intakeOff
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