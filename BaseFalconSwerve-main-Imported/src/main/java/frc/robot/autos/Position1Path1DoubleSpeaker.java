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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Position1Path1DoubleSpeaker extends SequentialCommandGroup implements IAutonomousPath6237MR {

    List<Trajectory> trajectoriesUsed = new ArrayList<Trajectory>();

    @Override
    public List<Trajectory> getTrajectoryList(){
        return trajectoriesUsed;
    }

    public Position1Path1DoubleSpeaker(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory movementTrajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(.1, .1)),
                new Pose2d(.2, .2, Rotation2d.fromDegrees(-120)),
            config);

        Trajectory movementTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(-120)),
            List.of(new Translation2d(.6, 0)),
            new Pose2d(1.2, 0, Rotation2d.fromDegrees(-180)),
        config);

        Trajectory movementTrajectory3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(-120)),
            List.of(new Translation2d(.25, 0)),
            new Pose2d(.5, 0, Rotation2d.fromDegrees(-180)),
        config);

        Trajectory movementTrajectory4 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(-180)),
            List.of(new Translation2d(-.7, 0)),
            new Pose2d(-1.7, 0, Rotation2d.fromDegrees(0)),
        config);

        trajectoriesUsed.add(movementTrajectory1);
        trajectoriesUsed.add(movementTrajectory2);
        trajectoriesUsed.add(movementTrajectory3);
        trajectoriesUsed.add(movementTrajectory4);

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

        SwerveControllerCommand swerveCommand4 =
            new SwerveControllerCommand(
                movementTrajectory4,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(movementTrajectory1.getInitialPose())),
            swerveCommand1,
            swerveCommand2,
            swerveCommand3,
            swerveCommand4
        );
    }
}