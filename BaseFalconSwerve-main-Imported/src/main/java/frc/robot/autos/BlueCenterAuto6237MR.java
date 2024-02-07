package frc.robot.autos;

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
import frc.robot.commands.ArmToIntakePositionCommand6237MR;
import frc.robot.commands.ArmToScoringPostionCommand6237MR;
import frc.robot.commands.RunLauncherCommand6237MR;
import frc.robot.commands.StopRunningLauncher6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class BlueCenterAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    List<Trajectory> trajectoriesUsed = new ArrayList<Trajectory>();

    @Override
    public List<Trajectory> getTrajectoryList(){
        return trajectoriesUsed;
    }
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueCenterStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueCenterStartingPositionY;}

    public BlueCenterAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        /*
            (absolute position origin)
            1.8, 3.5
            starting absolutes
         1  2.522, 5.540
         (intake on) 
         2  4.628, 5.540
         (intake off)
         3 7.479, 5.661 (mid point to clear stage at 6.071, 6.756)
         */
        /*
         (modded - relatives)
            .722, 2.04
            2.828, 2.04
            5.679, 2.161  (4.271, 3.256)
         */

        // An example trajectory to follow.  All units in meters.
        Trajectory movementTrajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(0.4, 1.0)),
          new Pose2d(.772, 2.04, Rotation2d.fromDegrees(0)),
        config);

        Trajectory movementTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(.772, 2.04, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1.5, 2.04)),
            new Pose2d(2.828, 2.04, Rotation2d.fromDegrees(0)),
        config);

        Trajectory movementTrajectory3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2.828, 2.04, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(4.771, 3.256)),
            new Pose2d(6.279, 2.161, Rotation2d.fromDegrees(0)),
        config);

        trajectoriesUsed.add(movementTrajectory1);
        trajectoriesUsed.add(movementTrajectory2);
        trajectoriesUsed.add(movementTrajectory3);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveCommand1 =
            new SwerveControllerCommand(
                movementTrajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
                
                
        SwerveControllerCommand swerveCommand2 =
            new SwerveControllerCommand(
                movementTrajectory2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
                    
        Command fireLauncher = new RunLauncherCommand6237MR(launcher);
        Command armToLoadNotePosition = new ArmToIntakePositionCommand6237MR(arm);
        Command armToScoringPosition = new ArmToScoringPostionCommand6237MR(arm);
        Command retractIntake = intake.retract();
        Command feedLauncher = intake.feedLauncher(launcher);
        Command stopCommand = new StopRunningLauncher6237MR(launcher);
        

        SwerveControllerCommand swerveCommand3 =
            new SwerveControllerCommand(
                movementTrajectory3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            swerveCommand1,
            retractIntake,
            swerveCommand2,
            swerveCommand3
        );
    }
}
