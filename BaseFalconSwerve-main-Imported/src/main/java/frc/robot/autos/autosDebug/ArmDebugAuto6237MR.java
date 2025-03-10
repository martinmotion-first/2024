package frc.robot.autos.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
 
    public ArmDebugAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command pauseCommand = new WaitCommand(2);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);

        Command pauseCommand2 = new WaitCommand(2);
        Command moveArmToScoringPosition2 = new ArmToScoringPostionCommand6237MR(arm);
        addRequirements(arm);
        addCommands(
            moveArmToScoringPosition,
            pauseCommand,
            moveArmToIntakePosition,
            pauseCommand2,
            moveArmToScoringPosition2
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
