package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class MovementDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    
    public MovementDebugAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        Command movementOne = new MoveByMetersCommand6237MR(s_Swerve, 1, 1);
        Command movementTwo = new MoveByMetersCommand6237MR(s_Swerve, 0, -1);
        Command movementThree = new MoveByMetersCommand6237MR(s_Swerve, -1, 0);
        Command movementFour = new MoveByMetersCommand6237MR(s_Swerve, -1, -1);

        addCommands(
            movementOne,
            movementTwo,
            movementThree,
            movementFour
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
