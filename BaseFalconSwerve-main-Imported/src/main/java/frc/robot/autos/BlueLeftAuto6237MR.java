package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.FireLauncherCommand6237MR;
import frc.robot.commands.MoveByMetersCommand6237MR;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class BlueLeftAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueLeftStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueLeftStartingPositionY;}


    public BlueLeftAuto6237MR(SwerveSubsystem s_Swerve, IntakeSubsystem intake, ArmSubsystem arm, LauncherSubsystem launcher){
        /*
         start x = 1.716 , y = 7.032 
         absolute coords
            rotate to -120 deg
            fire launcher
            rotate to -180 (/180) degrees
            intake on
            move to 3.665, 7.032 (still at 180 degrees)
            intake off
            move to 7.467, 7.032

        relative coords
            rotate to -120 deg
            fire launcher
            rotate to -180 (/180) degrees
            intake on
            move to 1.949,  (still at 180 degrees)
            intake off
            move to 7.467, 7.032
         */
        Command rotateToFireAtSpeaker = new RotateInPlaceCommand6237MR(s_Swerve, -120);
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
        Command rotateToBackwardsFromStarting = new RotateInPlaceCommand6237MR(s_Swerve, -60);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command turnIntakeOn = new ArmToIntakePositionCommand6237MR(arm);
        Command movementOne = new MoveByMetersCommand6237MR(s_Swerve, 0, .772); 
        Command turnIntakeOff = new ArmToScoringPostionCommand6237MR(arm);
        Command movementThree = new MoveByMetersCommand6237MR(s_Swerve, 0, 5.751);

        addCommands(
            rotateToFireAtSpeaker
            ,moveArmToScoringPosition
            ,fireLauncherCommand
            ,rotateToBackwardsFromStarting
            ,moveArmToIntakePosition
            ,turnIntakeOn
            ,turnIntakeOff
            ,movementOne
            ,movementThree
        );
    }
}
