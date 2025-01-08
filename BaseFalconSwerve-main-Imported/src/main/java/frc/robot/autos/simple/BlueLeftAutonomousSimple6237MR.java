package frc.robot.autos.simple;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.FireLauncherCommand6237MR;
import frc.robot.commands.MoveToCoordinatesCommand;
import frc.robot.commands.ZeroGyroCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class BlueLeftAutonomousSimple6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueLeftStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueLeftStartingPositionY;}

    public BlueLeftAutonomousSimple6237MR(SwerveSubsystem s_Swerve, IntakeSubsystem intake, ArmSubsystem arm, LauncherSubsystem launcher){
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
        Command moveRobotOut = new MoveToCoordinatesCommand(s_Swerve, -2.0, -1.0);
        Command zeroGyroCommand = new ZeroGyroCommand6237MR(s_Swerve);

        addCommands(
            fireLauncherCommand,
            moveRobotOut,
            zeroGyroCommand
        );
    }
}
