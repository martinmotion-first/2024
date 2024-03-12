package frc.robot.autos.doublespeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.FireLauncherCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.MoveToCoordinatesCommand;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.commands.RunIntakeCommand6237MR;
import frc.robot.commands.StopIntakeCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class BlueLeftDoubleSpeakerAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueLeftStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueLeftStartingPositionY;}


    public BlueLeftDoubleSpeakerAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
        Command moveRobotBack = new MoveToCoordinatesCommand(s_Swerve, -1, 0); //-1 will need tweaked
        Command rotateForNoteGrab = new RotateInPlaceCommand6237MR(s_Swerve, -45);

        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command waitForPositioningArmPositioning1 = new WaitCommand(Constants.AutonomousModeConstants.kAutonomousArmWaitTime);
        Command turnIntakeOn = new RunIntakeCommand6237MR(intake);
        Command moveBackwardsToGrabNote = new MoveToCoordinatesCommand(s_Swerve, -2.828, 0); //-2.8 will need tweaked
        Command turnIntakeOff = new StopIntakeCommand6237MR(intake);

        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command moveRobotBackToScoreAgain = new MoveToCoordinatesCommand(s_Swerve, 2, 1); //These values are PURELY a test for the moment
        Command rotateToFiring = new RotateInPlaceCommand6237MR(s_Swerve, 45);
        Command fireLauncherCommand2 = new FireLauncherCommand6237MR(launcher, intake);
        
        addCommands(
        fireLauncherCommand
        ,moveRobotBack
        ,rotateForNoteGrab
        ,moveArmToIntakePosition
        ,waitForPositioningArmPositioning1 
        ,turnIntakeOn 
        ,moveBackwardsToGrabNote 
        ,turnIntakeOff 
        ,moveArmToScoringPosition 
        ,moveRobotBackToScoreAgain
        ,rotateToFiring 
        ,fireLauncherCommand2 
        );
    }
}
