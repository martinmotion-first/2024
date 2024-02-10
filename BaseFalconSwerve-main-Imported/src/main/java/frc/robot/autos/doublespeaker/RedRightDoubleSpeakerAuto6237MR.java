package frc.robot.autos.doublespeaker;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.autos.IAutonomousPath6237MR;
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

public class RedRightDoubleSpeakerAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueRightStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueRightStartingPositionY;}

    public RedRightDoubleSpeakerAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command moveRobotToFiringPosition = new MoveByMetersCommand6237MR(s_Swerve, 0.202, -2.253);
        Command rotateToFiringPostion = new RotateInPlaceCommand6237MR(s_Swerve, 120); //???
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command fireLauncherCommand = new RunLauncherCommand6237MR(launcher);
        Command stopLauncher = new StopRunningLauncher6237MR(launcher);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command rotateToNoteIntakePostion = new RotateInPlaceCommand6237MR(s_Swerve, 60); //???
        Command moveRobotToNotePosition = new MoveByMetersCommand6237MR(s_Swerve, 1.977, -2.129);
        //moveArmToScoringPosition
        Command moveRobotBackToFiringPosition = new MoveByMetersCommand6237MR(s_Swerve, -0.202, 2.253); //NOTE - this may need adjusting or removal
        //rotateToFiringPostion
        //fireLauncherCommand
        //stopLauncher
        Command rotateToLeave = new RotateInPlaceCommand6237MR(s_Swerve, 60); 
        Command moveRobotToLeave = new MoveByMetersCommand6237MR(s_Swerve, 5.796, -0.62); 

        addCommands(
            moveRobotToFiringPosition
            ,rotateToFiringPostion
            ,moveArmToScoringPosition
            ,fireLauncherCommand
            ,stopLauncher
            ,moveArmToIntakePosition
            ,rotateToNoteIntakePostion 
            ,moveRobotToNotePosition
            ,moveArmToScoringPosition
            ,moveRobotBackToFiringPosition
            ,rotateToFiringPostion
            ,fireLauncherCommand
            ,stopLauncher
            ,rotateToLeave
            ,moveRobotToLeave
        );
    }
}
