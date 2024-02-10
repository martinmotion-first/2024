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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

public class RedCenterAuto6237MR extends SequentialCommandGroup implements IAutonomousPath6237MR {
    // List<Trajectory> trajectoriesUsed = new ArrayList<Trajectory>();

    // @Override
    // public List<Trajectory> getTrajectoryList(){
    //     return trajectoriesUsed;
    // }
    @Override
    public double getSimulatorDisplayCoordinateX(){return SimulatorConstants6237MR.kRedCenterStartingPositionX;}
    @Override
    public double getSimulatorDisplayCoordinateY(){return SimulatorConstants6237MR.kRedCenterStartingPositionY;}

    public RedCenterAuto6237MR(Swerve s_Swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory movementTrajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(0.101, -1.1265)),
          new Pose2d(.772,-2.04, Rotation2d.fromDegrees(0)),
        config);

        Trajectory movementTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(.772, -2.04, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(0.6785, -2.191)),
            new Pose2d(2.828, -2.04, Rotation2d.fromDegrees(0)),
        config);

        Trajectory movementTrajectory3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2.828, -2.04, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1.566, -2.129)),
            new Pose2d(5.679, -2.161, Rotation2d.fromDegrees(0)),
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
            // intakeOn,
            swerveCommand2,
            // intakeOff,
            swerveCommand3
        );
    }
}
