package frc.robot.commands;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.DisplayUtil;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateInPlaceCommand6237MR extends Command{
    // private SwerveControllerCommand swerveCommand;

    private final SwerveSubsystem m_swerveDrive;
    private double m_desiredRotation;

    //Not _positive_ yet, but I believe that when the angle is received in the swerve command, 
    //it will be in the syntax of 0 to -180 for the right turning arc
    //and 0 to 180 for the left turning arc

    //REMEMBER! !!!!!!! THIS SHOULD BE REMOVED AND REPLACE WITH A PRIVATE CONSTRUCTOR IF THE FACTORY METHOD STYLE ENDS UP BEING USED!
    public RotateInPlaceCommand6237MR(SwerveSubsystem swerveDrive, double desiredRotation){
        m_swerveDrive = swerveDrive;
        m_desiredRotation = desiredRotation;

        // TrajectoryConfig config =
        //     new TrajectoryConfig(
        //             Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //         .setKinematics(Constants.Swerve.swerveKinematics);
        
        // var thetaController =
        //     new ProfiledPIDController(
        //         Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // List<Pose2d> waypoints = new ArrayList<Pose2d>();
        // waypoints.add(new Pose2d(0,0,new Rotation2d()));
        // Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config);

        // wrappedCommand = new SwerveControllerCommand(
        //     t,
        //     swerveDrive::getPose,
        //     Constants.Swerve.swerveKinematics,
        //     new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //     new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     swerveDrive::setModuleStates,
        //     swerveDrive);

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        // super.initialize(); 
        m_swerveDrive.zeroGyro();
        DisplayUtil.log(this.getName(), "In initialize ");
    }
    
    @Override
    public void execute() {
        // !!!! IF THIS WORKS !!!! - Remember to create a different version of drive for Autonomous potentially since this currently _could_ end up using Teleop constants...
        m_swerveDrive.driveAlternate(new Translation2d(), m_desiredRotation, false, true, .1);
        DisplayUtil.log(this.getName(), "In execute");
    }

    @Override
    public boolean isFinished(){
        boolean areWeFinished = Math.abs(m_swerveDrive.getYaw().getDegrees()) > Math.abs(m_desiredRotation);
        DisplayUtil.log(this.getName(), "In isFinished with value:" + areWeFinished);
        DisplayUtil.log(this.getName(), "Swerve subsytem yaw value:" + m_swerveDrive.getYaw().getDegrees());
        return areWeFinished;
    }

    @Override
    public void end(boolean interrupted) {
        DisplayUtil.log(this.getName(), "In end with interrupted variable: " + interrupted + "<<<<<");
    }

    public static Command Create(SwerveSubsystem swerve, double desiredRotation){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutonomousModeConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutonomousModeConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutonomousModeConstants.kPThetaController, 0, 0, Constants.AutonomousModeConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0,0,new Rotation2d(desiredRotation)));
        Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config);

        return new SwerveControllerCommand(
            t,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutonomousModeConstants.kPXController, 0, 0),
            new PIDController(Constants.AutonomousModeConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
    }


    //I found this "inplaceTurn" from a team that did swerve and provided some very useful (additional) info as well
    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
    // public void inplaceTurn(double power)
    // {
    //     leftFrontWheel.setDirection(135.0);
    //     leftBackWheel.setDirection(45.0);
    //     rightFrontWheel.setDirection(-45.0);
    //     rightBackWheel.setDirection(-135.0);

    //     leftFrontWheel.setSpeed(power);
    //     leftBackWheel.setSpeed(power);
    //     rightFrontWheel.setSpeed(power);
    //     rightBackWheel.setSpeed(power);
    // }
}
