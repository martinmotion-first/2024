package frc.robot.autosDebug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.commands.RotateInPlaceCommand6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AngleDebugAuto6237MR2 extends SequentialCommandGroup implements IAutonomousPath6237MR {

    public AngleDebugAuto6237MR2(SwerveSubsystem s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        Command rotateCommand = new RotateInPlaceCommand6237MR(s_Swerve, -120);
        addRequirements(s_Swerve);
        addCommands(rotateCommand);
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
