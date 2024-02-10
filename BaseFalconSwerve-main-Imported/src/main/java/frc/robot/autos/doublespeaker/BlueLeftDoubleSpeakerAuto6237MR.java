package frc.robot.autos.doublespeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.commands.RunLauncherCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class BlueLeftDoubleSpeakerAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueLeftStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueLeftStartingPositionY;}


    public BlueLeftDoubleSpeakerAuto6237MR(Swerve s_Swerve, IntakeSubsystem intake, ArmSubsystem arm, LauncherSubsystem launcher){
        Command rotateToFireAtSpeaker = new RotateInPlaceCommand6237MR(s_Swerve, -120);
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command fireLauncherCommand = new RunLauncherCommand6237MR(launcher);
        Command rotateToBackwardsFromStarting = new RotateInPlaceCommand6237MR(s_Swerve, -60);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command turnIntakeOn = new ArmToIntakePositionCommand6237MR(arm);
        Command movementOne = new MoveByMetersCommand6237MR(s_Swerve, 0, .772); 
        Command turnIntakeOff = new ArmToScoringPostionCommand6237MR(arm);
        Command movementTwo = new MoveByMetersCommand6237MR(s_Swerve, 0, .772); //QUESTION - is this forward or should this be +.772 on the actual bot
        Command fireLauncherCommand2 = new RunLauncherCommand6237MR(launcher);
        Command movementThree = new MoveByMetersCommand6237MR(s_Swerve, 0, 5.751);

        addCommands(
            rotateToFireAtSpeaker
            ,moveArmToScoringPosition
            ,fireLauncherCommand
            ,rotateToBackwardsFromStarting
            ,moveArmToIntakePosition
            ,turnIntakeOn
            ,movementOne
            ,turnIntakeOff
            ,movementTwo
            ,rotateToFireAtSpeaker
            ,moveArmToScoringPosition
            ,fireLauncherCommand2
            ,movementThree
        );
    }
}
