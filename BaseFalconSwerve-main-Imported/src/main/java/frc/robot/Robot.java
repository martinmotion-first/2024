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
import frc.lib.util.DisplayUtil;
import frc.robot.autos.AutonomousModeChoices;
import frc.robot.autos.BlueRightAuto6237MR;
import frc.robot.autos.ExampleAutonomous;
import frc.robot.autos.IAutonomousPath6237MR;
import frc.robot.autos.Position1Path1DoubleSpeaker;
import frc.robot.autos.Position1Path1SpeakerShotGrabRingAndOut;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private static boolean ENABLE_LOGGING = false;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final Field2d m_field3 = new Field2d();
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

    chooserMenu.addOption("Example Auto", AutonomousModeChoices.EXAMPLE_AUTO.toString());
    chooserMenu.addOption("Example Auto w Field", AutonomousModeChoices.POSITION1_PATH1_SPEAKER_SHOT_GRAB_RING_LEAVE_PARKED.toString());
    chooserMenu.addOption("Position 1 - Path 1  (double speaker)", AutonomousModeChoices.POSITION1_PATH1_DOUBLE_SPEAKER.toString());

    chooserMenu.setDefaultOption("Example Auto", AutonomousModeChoices.EXAMPLE_AUTO.toString());

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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    log("ENTERING simulationInit");
    




    log("LEAVING simulationInit");
  }

  Command storedSimulationCommand = null;

  @Override
  public void simulationPeriodic(){
    log("ENTERING simulationPeriodic");
    CommandScheduler.getInstance().run();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
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
    //TEMP REMOVING
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    //END TEMP REMOVING


String selectedOption = chooserMenu.getSelected();
IAutonomousPath6237MR selectedCommandGroup;
    switch (AutonomousModeChoices.valueOf(selectedOption)){
      case EXAMPLE_AUTO:
        selectedCommandGroup = new ExampleAutonomous(m_robotContainer.getSwerve());
        break;
      // case EXAMPLE_AUTO_WITH_FIELD:
      //   selectedCommand = new ExampleAutotonomousWithField2d(m_robotContainer.getSwerve(), m_field);
      //   break;
      // case PLOT_SCRATCH_AUTO:
      //   selectedCommand = new PlotScratchAutonomous(m_robotContainer.getSwerve(), m_field);
      //   break;
      case POSITION1_PATH1_SPEAKER_SHOT_GRAB_RING_LEAVE_PARKED:
        selectedCommandGroup = new Position1Path1SpeakerShotGrabRingAndOut(m_robotContainer.getSwerve());
      case POSITION1_PATH1_DOUBLE_SPEAKER:
        selectedCommandGroup = new Position1Path1DoubleSpeaker(m_robotContainer.getSwerve());
        break;
      default:
        selectedCommandGroup = new ExampleAutonomous(m_robotContainer.getSwerve());
    }
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_field);
    // m_autonomousCommand = selectedCommandGroup;
    // if (m_autonomousCommand != null) {
    //   log("Scheduling command:" + m_autonomousCommand.getName());
    //   m_autonomousCommand.schedule();
    // }
    
    
    SmartDashboard.putData("Field 3", m_field3);


    //**** ALL OF THIS IS FOR TESTING WITHIN THE SIMULATOR */
    IAutonomousPath6237MR pathToTest = new BlueRightAuto6237MR(m_robotContainer.getSwerve());//new ExampleAutonomous(m_robotContainer.getSwerve());
    List<Trajectory> originalTrajectories = pathToTest.getTrajectoryList();
    //************ CANNOT POSSIBLY STRESS THIS ENOUGH **********
    /*
      THE FOLLOWING ADJUSTMENTS ARE DONE PURELY TO CENTER THE TRAJECTORIES TO A LOCATION FOR DISPLAY AND SHOULD ONLY BE DONE IN TEST/SIMULATION
    */
    double DISPLAY_OFFSET_TO_USE_AS_ORIGIN_X = 15.741;
    double DISPLAY_OFFSET_TO_USE_AS_ORIGIN_Y = 2.187;
    List<Trajectory> modifiedTrajectories = new ArrayList<Trajectory>();
    originalTrajectories.forEach((traj) -> {
      modifiedTrajectories.add(DisplayUtil.offsetTrajectoryCoordinatesForDisplayByXAndY(traj, DISPLAY_OFFSET_TO_USE_AS_ORIGIN_X, DISPLAY_OFFSET_TO_USE_AS_ORIGIN_Y));
    });
    // modifiedTrajectories = originalTrajectories;
    
    //************ END CANNOT POSSIBLY STRESS THIS ENOUGH **********

    // int counter = 0;
    modifiedTrajectories.forEach((trajectoryInList) -> {
      String randomName = java.util.UUID.randomUUID().toString();
      // String name = "trajectory";
      FieldObject2d objectToPlaceTrajectoryOn = m_field3.getObject(randomName);
      objectToPlaceTrajectoryOn.setTrajectory(trajectoryInList);

    });
    //***** ALL OF THE ABOVE IS FOR TESTING WITHIN THE SIMULATOR */
    

    log("LEAVING autonomousInit");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    log("ENTERING autonomousPeriodic");
    log("LEAVING autonomousPeriodic");
  }

  @Override
  public void teleopInit() {
    log("ENTERING teleopInit");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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
