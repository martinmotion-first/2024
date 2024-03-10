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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.DisplayUtil;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveByMetersCommand6237MR extends Command{
    private final SwerveControllerCommand m_swerveCommand;
    
    public MoveByMetersCommand6237MR(SwerveSubsystem swerveDrive, double xInMeters, double yInMeters){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(xInMeters / 2.0, yInMeters / 2.0, new Rotation2d()));
        Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config);

        m_swerveCommand = new SwerveControllerCommand(
            t,
            swerveDrive::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveDrive::setModuleStates,
            swerveDrive);

        
        addRequirements(swerveDrive);
        DisplayUtil.log(this.getName(), "Ending constructor");
    }

    public static SwerveControllerCommand FactoryIt(SwerveSubsystem swerveDrive, double xInMeters, double yInMeters){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        List<Pose2d> waypoints = new ArrayList<>();
        // waypoints.add(new Pose2d(0,0,new Rotation2d(0)));
        // waypoints.add(new Pose2d(1, -1, new Rotation2d(0)));
        waypoints.add(new Pose2d(xInMeters / 3, yInMeters / 3,new Rotation2d(0)));
        waypoints.add(new Pose2d(xInMeters / 2, yInMeters / 2, new Rotation2d(0)));
        Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config);

        SwerveControllerCommand newCommand = new SwerveControllerCommand(
            t,
            swerveDrive::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveDrive::setModuleStates,
            swerveDrive);

        return newCommand;
    }

    @Override
    public void execute(){
        DisplayUtil.log(this.getName(), "Starting execute");
        m_swerveCommand.execute();
        DisplayUtil.log(this.getName(), "Ending execute");
    }

    @Override
    public void initialize() {
        m_swerveCommand.initialize();
        DisplayUtil.log(this.getName(), "In initialize");
    }

    @Override
    public boolean isFinished()
    {   
        boolean f = m_swerveCommand.isFinished();
        DisplayUtil.log(this.getName(), "In isFinished with response:" + f);
        return f; //or false for the moment?
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        DisplayUtil.log(this.getName(), "In end with interrupted variable: " + interrupted + "<<<<<");
    }
}
