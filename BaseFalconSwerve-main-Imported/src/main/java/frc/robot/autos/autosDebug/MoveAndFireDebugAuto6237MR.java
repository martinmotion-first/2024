package frc.robot.autos.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.FireLauncherCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.MoveToCoordinatesCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveAndFireDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
 
    public MoveAndFireDebugAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        // Command movementOne = MoveByMetersCommand6237MR.Create(s_Swerve, 1, 1);
        Command movementOne = new MoveToCoordinatesCommand(s_Swerve, 1, 1);
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
        
        addCommands(
            movementOne
            ,fireLauncherCommand
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
