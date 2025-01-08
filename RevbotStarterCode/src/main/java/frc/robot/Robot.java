// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.DisplayUtility;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d(); //added - not to be here long term

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void simulationInit(){
      SmartDashboard.putData("Field", m_field);//I added this - it should not go here long term I don't think
          //following from https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
    //COPYING THIS AS IS FROM RobotContainer.getAutonomousCommand() FOR TESTING ONLY
        TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    Trajectory exampleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(4, 1), new Translation2d(5, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0)),
            config);
    //END COPYING
    //************ CANNOT POSSIBLY STRESS THIS ENOUGH **********
    Trajectory movementTrajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(new Translation2d(0.101, 1.1265)),
          new Pose2d(0.202, 2.253, Rotation2d.fromDegrees(120)),
      config);

    Trajectory movementTrajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0.202, 2.253, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(0.6785, 2.191)),
      // List.of(new Translation2d(.21, 2.19)),
      new Pose2d(1.155, 2.129, Rotation2d.fromDegrees(60)),
    config);

    Trajectory movementTrajectory3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.155, 2.129, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1.566, 2.129)),
      new Pose2d(1.977, 2.129, Rotation2d.fromDegrees(0)),
    config);

    Trajectory movementTrajectory4 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.977, 2.129, Rotation2d.fromDegrees(0
      )),
      List.of(new Translation2d(1.977, 0.62)),
      new Pose2d(5.796, 0.62, Rotation2d.fromDegrees(0)),
    config);
    
    double DISPLAY_OFFSET_X = 3;
    double DISPLAY_OFFSET_Y = 3;
    movementTrajectory1 = DisplayUtility.offsetTrajectoryCoordinatesForDisplayByXAndY(movementTrajectory1, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
    movementTrajectory2 = DisplayUtility.offsetTrajectoryCoordinatesForDisplayByXAndY(movementTrajectory2, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
    movementTrajectory3 = DisplayUtility.offsetTrajectoryCoordinatesForDisplayByXAndY(movementTrajectory3, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
    movementTrajectory4 = DisplayUtility.offsetTrajectoryCoordinatesForDisplayByXAndY(movementTrajectory4, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);

    /* 
      THE FOLLOWING ADJUSTMENTS ARE DONE PURELY TO CENTER THE TRAJECTORIES IN THE MIDDLE OF THE DISPLAY AND SHOULD ONLY BE DONE IN TEST/SIMULATION
    */
    // double DISPLAY_OFFSET_X = 3;
    // double DISPLAY_OFFSET_Y = 3;
    // exampleTrajectory = DisplayUtility.offsetTrajectoryCoordinatesForDisplayByXAndY(exampleTrajectory, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
    // exampleTrajectory2 = DisplayUtility.offsetTrajectoryCoordinatesForDisplayByXAndY(exampleTrajectory2, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
    //************ END CANNOT POSSIBLY STRESS THIS ENOUGH ********** 

    // m_field.getObject("traj").setTrajectory(exampleTrajectory);
    // m_field.getObject("traj2").setTrajectory(exampleTrajectory2);

    m_field.getObject("traj").setTrajectory(movementTrajectory1);
    m_field.getObject("traj2").setTrajectory(movementTrajectory2);
    m_field.getObject("traj3").setTrajectory(movementTrajectory3);
    m_field.getObject("traj4").setTrajectory(movementTrajectory4);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
