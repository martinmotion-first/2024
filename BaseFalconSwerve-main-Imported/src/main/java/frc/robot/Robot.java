// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutonomousModeChoices;
import frc.robot.autos.ExampleAutonomous;
import frc.robot.autos.ExampleAutotonomousWithField2d;
import frc.robot.autos.PlotScratchAutonomous;
import frc.robot.autos.Position1Path1DoubleSpeaker;
import frc.robot.subsystems.*;


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
  private final Field2d m_field = new Field2d();
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
    SmartDashboard.putData("Field", m_field);

    chooserMenu.setDefaultOption("Example Auto", AutonomousModeChoices.EXAMPLE_AUTO.toString());
    chooserMenu.addOption("Example Auto", AutonomousModeChoices.EXAMPLE_AUTO.toString());
    chooserMenu.addOption("Example Auto w Field", AutonomousModeChoices.EXAMPLE_AUTO_WITH_FIELD.toString());
    chooserMenu.addOption("Plot Scratch", AutonomousModeChoices.PLOT_SCRATCH_AUTO.toString());
    chooserMenu.addOption("Position 1 - Path 1  (double speaker)", AutonomousModeChoices.POSITION1_PATH1_DOUBLE_SPEAKER.toString());
    SmartDashboard.putData("Auto choices", chooserMenu);
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
    m_field.setRobotPose(m_robotContainer.retrieveOdometry().getPoseMeters());
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
    String selectedOption = chooserMenu.getSelected();
    Command selectedCommand;
    switch (AutonomousModeChoices.valueOf(selectedOption)){
      case EXAMPLE_AUTO:
        selectedCommand = new ExampleAutonomous(m_robotContainer.getSwerve());
        break;
      case EXAMPLE_AUTO_WITH_FIELD:
        selectedCommand = new ExampleAutotonomousWithField2d(m_robotContainer.getSwerve(), m_field);
        break;
      case PLOT_SCRATCH_AUTO:
        selectedCommand = new PlotScratchAutonomous(m_robotContainer.getSwerve(), m_field);
        break;
      case POSITION1_PATH1_DOUBLE_SPEAKER:
        selectedCommand = new Position1Path1DoubleSpeaker(m_robotContainer.getSwerve());
        break;
      default:
        selectedCommand = new ExampleAutonomous(m_robotContainer.getSwerve());
    }
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_field);
    m_autonomousCommand = selectedCommand;
    if (m_autonomousCommand != null) {
      log("Scheduling command:" + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }
    log("LEAVING simulationInit");
  }

  Command storedSimulationCommand = null;

  @Override
  public void simulationPeriodic(){
    log("ENTERING simulationPeriodic");
    CommandScheduler.getInstance().run();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    //THIS IS HIGHLY TEMPORARY, BUT ATTEMPTING TO FORCE A MOVE COMMAND
    if(storedSimulationCommand == null){
      storedSimulationCommand = new PlotScratchAutonomous(m_robotContainer.getSwerve(), m_field);
    }
    if(!CommandScheduler.getInstance().isScheduled(storedSimulationCommand)){
      log("Scheduling a new Plot Command");
      CommandScheduler.getInstance().schedule(storedSimulationCommand);
      //EVEN MORE TEMP - THIS SEEMS LIKE A TERRIBLE IDEA, BUT ATTEMPTING TO MANUALLY EXECUTE
      log("################################### EXECUTING A PLOT COMMAND ###########################################");
      // storedSimulationCommand.execute();
      log("################################### POST EXECUTING A PLOT COMMAND ###########################################");
    }
    //END TEMPORARY
    Pose2d coords = m_robotContainer.retrieveOdometry().getPoseMeters();
    log("X(meters)=" + coords.getX());
    log("Y(meters)=" + coords.getX());
    m_field.setRobotPose(coords);
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
