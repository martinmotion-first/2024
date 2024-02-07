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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class AnglePlaygroundAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    List<Trajectory> trajectoriesUsed = new ArrayList<Trajectory>();
    @Override
    public List<Trajectory> getTrajectoryList(){
        return trajectoriesUsed;
    }
    @Override
    public double getSimulatorDisplayCoordinateX(){return 4;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return 2;}
    

    public AnglePlaygroundAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory movementTrajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 20, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(10, 20)),
          new Pose2d(15, 20, Rotation2d.fromDegrees(45)),
        config);

        Trajectory movementTrajectory2 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 17, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(10, 17)),
          new Pose2d(15, 17, Rotation2d.fromDegrees(-45)),
        config);

        Trajectory movementTrajectory1ManyWaypoints = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(120)),
          List.of(
                    // new Translation2d(1, Rotation2d.fromDegrees(0))
                    // new Translation2d(2, Rotation2d.fromDegrees(0)), 
                    // new Translation2d(3, Rotation2d.fromDegrees(0)),
                    // new Translation2d(4, Rotation2d.fromDegrees(0))
                    // new Translation2d(2, 0),
                     new Translation2d(4, 0)
                    //  new Translation2d(6, 0),
                    //  new Translation2d(8, 0)
                ),
          new Pose2d(10, 0, Rotation2d.fromDegrees(120)),
        config);

        // Trajectory movementTrajectory2ManyWaypoints = TrajectoryGenerator.generateTrajectory(
        //   new Pose2d(0, 17, Rotation2d.fromDegrees(0)),
        //   List.of(new Translation2d(10, 17)),
        //   new Pose2d(20, 17, Rotation2d.fromDegrees(-45)),
        // config);

        // trajectoriesUsed.add(movementTrajectory1);
        // trajectoriesUsed.add(movementTrajectory2);
        trajectoriesUsed.add(movementTrajectory1ManyWaypoints);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand irrelevantCommand =
            new SwerveControllerCommand(
                movementTrajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            irrelevantCommand      
        );
    }

}
