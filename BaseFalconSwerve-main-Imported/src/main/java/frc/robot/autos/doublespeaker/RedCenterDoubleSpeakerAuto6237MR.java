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

public class RedCenterDoubleSpeakerAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueCenterStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueCenterStartingPositionY;}

    public RedCenterDoubleSpeakerAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command moveToWhereFirstRingGetsFired = new MoveByMetersCommand6237MR(s_Swerve, .772, -2.04);
        Command moveArmToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command rotateRobotToFiringAngle = new RotateInPlaceCommand6237MR(s_Swerve, -180);
        Command fireLauncherCommand = new RunLauncherCommand6237MR(launcher);
        Command rotateToIntakePostion = new RotateInPlaceCommand6237MR(s_Swerve, -180);
        Command moveArmToIntakePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command turnIntakeOn = new ArmToIntakePositionCommand6237MR(arm);
        Command moveRobotToNote = new MoveByMetersCommand6237MR(s_Swerve, 2.828, -2.04);
        Command turnIntakeOff = new ArmToScoringPostionCommand6237MR(arm);
        //moveArmToScoringPosition
        Command moveToSecondFiringPosition = new MoveByMetersCommand6237MR(s_Swerve, .772, -2.04);// NOTE - THESE ARE A PLACEHOLDER
        //rotateRobotToFiringAngle
        //fireLauncher
        Command rotateToLeave = new RotateInPlaceCommand6237MR(s_Swerve, 180); //this may be neither needed, nor desirable
        Command moveRobotToLeave = new MoveByMetersCommand6237MR(s_Swerve, 5.679, -2.161);

        addCommands(
            moveToWhereFirstRingGetsFired 
            ,moveArmToScoringPosition 
            ,rotateRobotToFiringAngle 
            ,fireLauncherCommand 
            ,rotateToIntakePostion 
            ,moveArmToIntakePosition 
            ,turnIntakeOn 
            ,moveRobotToNote 
            ,turnIntakeOff 
            ,moveArmToScoringPosition
            ,moveToSecondFiringPosition 
            ,rotateRobotToFiringAngle
            ,fireLauncherCommand
            ,rotateToLeave 
            ,moveRobotToLeave 
        );
    }
}
