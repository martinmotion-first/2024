package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AngleDebugAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {

    public AngleDebugAuto6237MR(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){

        //Attempt 1 utilizing the existing Command structure...
        Command rotateCommand = new RotateInPlaceCommand6237MR(s_Swerve, 120);
        Command rotateCommand2 = new RotateInPlaceCommand6237MR(s_Swerve, -30);
        Command rotateCommand3 = new RotateInPlaceCommand6237MR(s_Swerve, 270);
        //Attempt 2 using a Factory style build to retrieve the command
        // Command rotateCommand = RotateInPlaceCommand6237MR.Create(s_Swerve, 120);
        
        addCommands(rotateCommand
        ,rotateCommand2,
        rotateCommand3);
    }

    @Override
    public double getSimulatorDisplayCoordinateX() {
        return 0.0;
    }

    @Override
    public double getSimulatorDisplayCoordinateY() {
        return 0.0;
    }
    
}
