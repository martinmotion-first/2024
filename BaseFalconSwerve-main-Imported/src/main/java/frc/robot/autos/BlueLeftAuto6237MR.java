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
import frc.robot.Constants.SimulatorConstants6237MR;
import frc.robot.commands.RotationOnlyCommand6237MR;
import frc.robot.subsystems.Swerve;

public class BlueLeftAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    List<Trajectory> trajectoriesUsed = new ArrayList<Trajectory>();
    @Override
    public List<Trajectory> getTrajectoryList(){
        return trajectoriesUsed;
    }
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kBlueLeftStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kBlueLeftStartingPositionY;}


    public BlueLeftAuto6237MR(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

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

        Trajectory movementTrajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(0.4, 0)),
          new Pose2d(.772, 0, Rotation2d.fromDegrees(0)),
        config);

        Trajectory movementTrajectory2 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(2.3, 0)),
          new Pose2d(5.751, 0, Rotation2d.fromDegrees(0)),
        config);

        trajectoriesUsed.add(movementTrajectory1);
        trajectoriesUsed.add(movementTrajectory2);


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
                movementTrajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            swerveCommand1
            ,swerveCommand2        
        );
    }
}
