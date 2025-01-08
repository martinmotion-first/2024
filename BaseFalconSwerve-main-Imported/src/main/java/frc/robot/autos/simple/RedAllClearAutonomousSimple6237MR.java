package frc.robot.autos.simple;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.MoveToCoordinatesCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RedAllClearAutonomousSimple6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return 0;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return 0;}

    public RedAllClearAutonomousSimple6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command moveRobotOut = new MoveToCoordinatesCommand(s_Swerve, -2.0, 0.0);

        addCommands(
            moveRobotOut
        );
    }
}
