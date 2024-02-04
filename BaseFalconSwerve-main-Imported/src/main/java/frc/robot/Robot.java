// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.DisplayUtil;
import frc.robot.autos.AutonomousModeChoices6237MR;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.autos.RedCenterAuto6237MR;
import frc.robot.autos.RedLeftAuto6237MR;
import frc.robot.autos.RedRightAuto6237MR;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private static boolean ENABLE_LOGGING = false;

  // private Command m_autonomousCommand;
  private SequentialCommandGroup m_autonomousCommandGroup;

  private RobotContainer m_robotContainer;
  private Field2d m_field3 = new Field2d();
  private SendableChooser<String> chooserMenu = new SendableChooser<String>();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    chooserMenu.addOption("Example Auto", AutonomousModeChoices6237MR.EXAMPLE_AUTO.toString());
    chooserMenu.addOption("Blue Right Auto Mode 1", AutonomousModeChoices6237MR.BLUE_RIGHT_AUTO_MODE_1.toString());
    chooserMenu.addOption("Blue Center Auto Mode 1", AutonomousModeChoices6237MR.BLUE_CENTER_AUTO_MODE_1.toString()); 
    chooserMenu.addOption("Blue Left Auto Mode 1", AutonomousModeChoices6237MR.BLUE_LEFT_AUTO_MODE_1.toString());
    chooserMenu.addOption("Red Left Auto Mode 1", AutonomousModeChoices6237MR.RED_LEFT_AUTO_MODE_1.toString()); //inverted Blue Right Auto
    chooserMenu.addOption("Red Center Auto Mode 1", AutonomousModeChoices6237MR.RED_CENTER_AUTO_MODE_1.toString()); //inverted Blue Center Auto
    chooserMenu.addOption("Red Right Auto Mode 1", AutonomousModeChoices6237MR.RED_RIGHT_AUTO_MODE_1.toString()); //inverted Blue Left Auto
    chooserMenu.addOption("Angle Playground", AutonomousModeChoices6237MR.ANGLE_PLAYGROUND.toString()); //inverted Blue Left Auto
    
    chooserMenu.setDefaultOption("Example Auto", AutonomousModeChoices6237MR.EXAMPLE_AUTO.toString());

    SmartDashboard.putData("Auto choices", chooserMenu);
    SmartDashboard.putData(m_robotContainer.getSwerve());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    log("ENTERING robotPeriodic");
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    m_field3.setRobotPose(m_robotContainer.retrieveOdometry().getPoseMeters());
    log("LEAVING robotPeriodic");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    log("ENTERING disabledInit");
    log("LEAVING disabledInit");
  }

  @Override
  public void simulationInit(){
    log("ENTERING simulationInit");
    SmartDashboard.putData("Field 3", m_field3);
    log("LEAVING simulationInit");
  }

  Command storedSimulationCommand = null;

  @Override
  public void simulationPeriodic(){
    log("ENTERING simulationPeriodic");
    CommandScheduler.getInstance().run();

    chooserMenu.getSelected();
    if(m_autonomousCommandGroup != null && !m_autonomousCommandGroup.isScheduled()){

      //SequentialCommandGroups are used for normal operation and is what autonomous paths should extend
      //For simulation purpose though, we are implementing the IAutonomousPath6237MR.java interface
      //This allows this cast to be safe and to do some unique operations that wouldn't make sense in a different context
      //such as creating absolute x,y coordinates and performing visual translations to make these paths appear correctly
      IAutonomousPath6237MR pathToTest = (IAutonomousPath6237MR) m_autonomousCommandGroup;

      //   switch (AutonomousModeChoices.valueOf(selectedOption)){
      //     case EXAMPLE_AUTO:
      //       m_autonomousCommandGroup = new ExampleAutonomous(m_robotContainer.getSwerve());
      //       m_autonomousCommandGroup.schedule();
      //       break;
      //     case BLUE_RIGHT_AUTO_MODE_1:
      //       m_autonomousCommandGroup = new BlueRightAuto6237MR(m_robotContainer.getSwerve());
      //       m_autonomousCommandGroup.schedule();
      //       break;
      //     case RED_LEFT_AUTO_MODE_1:
      //       m_autonomousCommandGroup = new RedLeftAuto6237MR(m_robotContainer.getSwerve());
      //       m_autonomousCommandGroup.schedule();
      //       break;
      //     default:
      //       m_autonomousCommandGroup = new ExampleAutonomous(m_robotContainer.getSwerve());
      //       m_autonomousCommandGroup.schedule();
      //   }
      // }

      List<Trajectory> originalTrajectories = pathToTest.getTrajectoryList();
      /*
        THE FOLLOWING ADJUSTMENTS ARE DONE PURELY TO CENTER THE TRAJECTORIES TO A LOCATION FOR DISPLAY AND SHOULD ONLY BE DONE IN TEST/SIMULATION
      */
      // List<Trajectory> modifiedTrajectories = new ArrayList<Trajectory>();
      // originalTrajectories.forEach((traj) -> {
      //   modifiedTrajectories.add(DisplayUtil.offsetTrajectoryCoordinatesForDisplayByXAndY(traj, pathToTest.getSimulatorDisplayCoordinateX(), pathToTest.getSimulatorDisplayCoordinateY()));
      // });
      List<Trajectory> modifiedTrajectories = new ArrayList<Trajectory>();
      if(m_autonomousCommandGroup instanceof RedLeftAuto6237MR || m_autonomousCommandGroup instanceof RedCenterAuto6237MR || m_autonomousCommandGroup instanceof RedRightAuto6237MR){
        originalTrajectories.forEach((traj) -> {
          Trajectory modifiedTrajectory = DisplayUtil.offsetTrajectoryCoordinatesForDisplayByXAndY(traj, pathToTest.getSimulatorDisplayCoordinateX(), pathToTest.getSimulatorDisplayCoordinateY());
          modifiedTrajectory = DisplayUtil.invertXValuesForRedStartingCoordinates(modifiedTrajectory);
          modifiedTrajectories.add(modifiedTrajectory);
        });
      }else{
        originalTrajectories.forEach((traj) -> {
          modifiedTrajectories.add(DisplayUtil.offsetTrajectoryCoordinatesForDisplayByXAndY(traj, pathToTest.getSimulatorDisplayCoordinateX(), pathToTest.getSimulatorDisplayCoordinateY()));
        });
      }

        // List<Trajectory> modifiedTrajectories2 = new ArrayList<Trajectory>();
        // originalTrajectories.forEach((traj2) -> {
        //   modifiedTrajectories2.add(DisplayUtil.invertXValuesForRedStartingCoordinates(traj2));
        // });
        // modifiedTrajectories = modifiedTrajectories2;

      // modifiedTrajectories.forEach((trajectoryInList) -> {
      //   String randomName = java.util.UUID.randomUUID().toString();
      //   FieldObject2d objectToPlaceTrajectoryOn = m_field3.getObject(randomName);
      //   objectToPlaceTrajectoryOn.setTrajectory(trajectoryInList);
      // });
      for(int trajectoryIndex = 0; trajectoryIndex < modifiedTrajectories.size(); trajectoryIndex++){
        String name = "trajectory" + trajectoryIndex;
        FieldObject2d objectToPlaceTrajectoryOn = m_field3.getObject(name);
        objectToPlaceTrajectoryOn.setTrajectory(modifiedTrajectories.get(trajectoryIndex));
      }
    }
    log("LEAVING simulationPeriodic");
  }

  @Override
  public void disabledPeriodic() {
    log("ENTERING disabledPeriodic");
    log("LEAVING disabledPeriodic");
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    log("ENTERING autonomousInit");
    m_autonomousCommandGroup = m_robotContainer.getAutonomousCommand(chooserMenu.getSelected());
    if (m_autonomousCommandGroup != null) {
      m_autonomousCommandGroup.schedule();
    }
    log("LEAVING autonomousInit");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    log("ENTERING autonomousPeriodic");
    // if(m_autonomousCommandGroup != null && !m_autonomousCommandGroup.isScheduled()){
    //   String selectedOption = chooserMenu.getSelected();
    //   switch (AutonomousModeChoices.valueOf(selectedOption)){
    //     case EXAMPLE_AUTO:
    //       m_autonomousCommandGroup = new ExampleAutonomous(m_robotContainer.getSwerve());
    //       m_autonomousCommandGroup.schedule();
    //       break;
    //     case BLUE_RIGHT_AUTO_MODE_1:
    //       m_autonomousCommandGroup = new BlueRightAuto6237MR(m_robotContainer.getSwerve());
    //       m_autonomousCommandGroup.schedule();
    //       break;
    //     case RED_LEFT_AUTO_MODE_1:
    //       m_autonomousCommandGroup = new RedLeftAuto6237MR(m_robotContainer.getSwerve());
    //       m_autonomousCommandGroup.schedule();
    //       break;
    //     default:
    //       m_autonomousCommandGroup = new ExampleAutonomous(m_robotContainer.getSwerve());
    //       m_autonomousCommandGroup.schedule();
    //   }
    // }
    log("LEAVING autonomousPeriodic");
  }

  @Override
  public void teleopInit() {
    log("ENTERING teleopInit");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommandGroup != null) {
      m_autonomousCommandGroup.cancel();
    }
    //Swerve swerveSubsystem = new Swerve();
    //swerveSubsystem.resetModulesToAbsolute();
    log("LEAVING teleopInit");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    log("ENTERING teleopInit");
    log("LEAVING teleopInit");
  }

  @Override
  public void testInit() {
    log("ENTERING testInit");
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    log("LEAVING testInit");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    log("ENTERING testPeriodic");
    log("LEAVING testPeriodic");
  }

  private void log(String message){
    if(ENABLE_LOGGING){
      System.out.println("***************************" + message + "***************************");
    }
  }
}
