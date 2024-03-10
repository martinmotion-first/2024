package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveByMetersCommand6237MR {
    
    private MoveByMetersCommand6237MR(){
        
    }

    public static SwerveControllerCommand Create(SwerveSubsystem swerveDrive, double xInMeters, double yInMeters){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutonomousModeConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutonomousModeConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutonomousModeConstants.kPThetaController, 0, 0, Constants.AutonomousModeConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        List<Pose2d> waypoints = new ArrayList<>();
        // waypoints.add(new Pose2d(xInMeters / 3, yInMeters / 3,new Rotation2d(0)));
        // waypoints.add(new Pose2d( (xInMeters * 2) / 3, (yInMeters * 2) / 3, new Rotation2d(0)));
        //a change to these waypoints that feels like a good idea but cant be tested at the moment....
        waypoints.add(new Pose2d(xInMeters / 3, yInMeters / 3,new Rotation2d(swerveDrive.getYaw().getDegrees())));
        waypoints.add(new Pose2d( (xInMeters * 2) / 3, (yInMeters * 2) / 3, new Rotation2d(swerveDrive.getYaw().getDegrees())));
        Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config);

        SwerveControllerCommand newCommand = new SwerveControllerCommand(
            t,
            swerveDrive::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutonomousModeConstants.kPXController, 0, 0),
            new PIDController(Constants.AutonomousModeConstants.kPYController, 0, 0),
            thetaController,
            swerveDrive::setModuleStates,
            swerveDrive);

        return newCommand;
    }
}
