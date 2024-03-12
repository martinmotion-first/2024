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
import frc.robot.commands.ZeroGyroCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class BlueCenterDoubleSpeakerAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueCenterStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueCenterStartingPositionY;}

    public BlueCenterDoubleSpeakerAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        // Command moveToWhereFirstRingGetsFired = MoveByMetersCommand6237MR.Create(s_Swerve, .772, 2.04);
        // Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        // Command rotateRobotToFiringAngle = new RotateInPlaceCommand6237MR(s_Swerve, -180);
        Command fireLauncherCommand = new FireLauncherCommand6237MR(launcher, intake);
        // Command rotateToIntakePostion = new RotateInPlaceCommand6237MR(s_Swerve, -180);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command waitForPositioningArmPositioning1 = new WaitCommand(Constants.AutonomousModeConstants.kAutonomousArmWaitTime);
        Command turnIntakeOn = new RunIntakeCommand6237MR(intake);
        // Command moveRobotToNote = MoveByMetersCommand6237MR.Create(s_Swerve, 2.828, 2.04);
        Command moveRobotToNote = new MoveToCoordinatesCommand(s_Swerve, -2.828, 0);
        Command turnIntakeOff = new StopIntakeCommand6237MR(intake);
        //moveArmToScoringPosition
        Command moveToSecondFiringPosition = new MoveToCoordinatesCommand(s_Swerve, 2.828, 0);// NOTE - THESE ARE A PLACEHOLDER
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command waitForArmPositioning2 = new WaitCommand(Constants.AutonomousModeConstants.kAutonomousArmWaitTime);
        Command fireLauncherCommand2 = new FireLauncherCommand6237MR(launcher, intake);
        //rotateRobotToFiringAngle
        //fireLauncher
        // Command rotateToLeave = new RotateInPlaceCommand6237MR(s_Swerve, 180); //this may be neither needed, nor desirable
        // Command moveRobotToLeave = MoveByMetersCommand6237MR.Create(s_Swerve, 5.679, 2.161);
        Command zeroGyroCommand = new ZeroGyroCommand6237MR(s_Swerve);


        addCommands(
            // moveToWhereFirstRingGetsFired 
            // ,moveArmToScoringPosition 
            // ,rotateRobotToFiringAngle 
            fireLauncherCommand 
            // ,rotateToIntakePostion 
            ,moveArmToIntakePosition 
            ,waitForPositioningArmPositioning1
            ,turnIntakeOn 
            ,moveRobotToNote 
            ,turnIntakeOff 
            ,moveToSecondFiringPosition 
            ,moveArmToScoringPosition
            ,waitForArmPositioning2
            ,fireLauncherCommand2
            ,zeroGyroCommand
            // ,rotateToLeave 
            // ,moveRobotToLeave 
        );
    }
}
