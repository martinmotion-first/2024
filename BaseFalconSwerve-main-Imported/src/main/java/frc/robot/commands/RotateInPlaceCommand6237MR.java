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
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RotateInPlaceCommand6237MR extends Command{
    private SwerveControllerCommand swerveCommand;

    //Not _positive_ yet, but I believe that when the angle is received in the swerve command, 
    //it will be in the syntax of 0 to -180 for the right turning arc
    //and 0 to 180 for the left turning arc
    public RotateInPlaceCommand6237MR(Swerve swerveDrive, double desiredRotation){
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
        waypoints.add(new Pose2d(0,0,new Rotation2d()));
        Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config);

        swerveCommand = new SwerveControllerCommand(
            t,
            swerveDrive::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveDrive::setModuleStates,
            swerveDrive);

        /*
         *   public SwerveControllerCommand(
                Trajectory trajectory,
                Supplier<Pose2d> pose,
                SwerveDriveKinematics kinematics,
                PIDController xController,
                PIDController yController,
                ProfiledPIDController thetaController,
                Consumer<SwerveModuleState[]> outputModuleStates,
                Subsystem... requirements) {
         */

        thetaController.enableContinuousInput(-180, 180); // Ensure continuous input for headings
        /* Thanks ChatGPT!
        super(
            new TrapezoidProfile.Constraints(maxVelocity, 1.0), // Adjust acceleration as needed
            new PIDController(kP, kI, kD),
            swerveDrive::getHeading,
            () -> 90.0, // Target heading for a 90-degree turn
            output -> swerveDrive.drive(0.0, 0.0, output, false), // Adjust based on your swerve drive implementation
            swerveDrive
        );

        getController().enableContinuousInput(-180, 180); // Ensure continuous input for headings
        */
    }
    
    @Override
    public void execute() {
        swerveCommand.execute();

        // m_Swerve.getModulePositions()[0].angle = Rotation2d.fromDegrees(135.0); // no idea how to know which of thse module positions represents what wheel at this moment
        // m_Swerve.getModulePositions()[1].angle = Rotation2d.fromDegrees(45.0);  // if this should even involve the module positions and not the module states
        // m_Swerve.getModulePositions()[2].angle = Rotation2d.fromDegrees(-45.0); 
        // m_Swerve.getModulePositions()[3].angle = Rotation2d.fromDegrees(-135.0);
        
    //     leftFrontWheel.setDirection(135.0);
    //     leftBackWheel.setDirection(45.0);
    //     rightFrontWheel.setDirection(-45.0);
    //     rightBackWheel.setDirection(-135.0);

    //     leftFrontWheel.setSpeed(power);
    //     leftBackWheel.setSpeed(power);
    //     rightFrontWheel.setSpeed(power);
    //     rightBackWheel.setSpeed(power);
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
