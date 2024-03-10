package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
 
    public IntakeDebugAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        Command turnIntakeOn = intake.feedLauncher(launcher);
        Command pauseCommand = new WaitCommand(2);
        Command intakeOff = new RunCommand(() -> intake.setPower(0.0), intake);
        addRequirements(intake, launcher);
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