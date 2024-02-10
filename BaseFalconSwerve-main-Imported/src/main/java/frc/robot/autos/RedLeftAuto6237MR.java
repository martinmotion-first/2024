package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.commands.RunLauncherCommand6237MR;
import frc.robot.commands.StopRunningLauncher6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class RedLeftAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kRedLeftStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kRedLeftStartingPositionY;}

    public RedLeftAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command moveRobotToFiringPosition = new MoveByMetersCommand6237MR(s_Swerve, 0.202, -2.253);
        Command rotateToFiringPostion = new RotateInPlaceCommand6237MR(s_Swerve, 120); //???
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command fireLauncherCommand = new RunLauncherCommand6237MR(launcher);
        Command stopLauncher = new StopRunningLauncher6237MR(launcher);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command rotateToLeave = new RotateInPlaceCommand6237MR(s_Swerve, 60); 
        Command moveRobotToLeave = new MoveByMetersCommand6237MR(s_Swerve, 5.796, -0.62); 

        addCommands(
            moveRobotToFiringPosition
            ,rotateToFiringPostion
            ,moveArmToScoringPosition
            ,fireLauncherCommand
            ,stopLauncher
            ,moveArmToIntakePosition
            ,rotateToLeave
            ,moveRobotToLeave
        );
    }
}
