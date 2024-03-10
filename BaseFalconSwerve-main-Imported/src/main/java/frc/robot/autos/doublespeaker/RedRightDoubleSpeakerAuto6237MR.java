package frc.robot.autos.doublespeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.FireLauncherCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RedRightDoubleSpeakerAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueRightStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueRightStartingPositionY;}

    public RedRightDoubleSpeakerAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command moveRobotToFiringPosition = new MoveByMetersCommand6237MR(s_Swerve, 0.202, -2.253);
        Command rotateToFiringPostion = new RotateInPlaceCommand6237MR(s_Swerve, 120); //???
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
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
            ,moveArmToIntakePosition
            ,rotateToNoteIntakePostion 
            ,moveRobotToNotePosition
            ,moveArmToScoringPosition
            ,moveRobotBackToFiringPosition
            ,rotateToFiringPostion
            ,fireLauncherCommand
            ,rotateToLeave
            ,moveRobotToLeave
        );
    }
}
