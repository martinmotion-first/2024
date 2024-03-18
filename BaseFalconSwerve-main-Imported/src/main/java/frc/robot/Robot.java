// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.AutonomousModeChoices6237MR;
import edu.wpi.first.cameraserver.CameraServer;

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
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    //!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!!!
    // I am currently removing these autonomous options as they'll need to be investigated again and tested against.
    // Mostly I don't want someone to choose one accidentally during testing and tuning

    chooserMenu.addOption("Blue Left Auto Mode 1", AutonomousModeChoices6237MR.BLUE_LEFT_AUTO_MODE_1.toString());
    chooserMenu.addOption("Blue Center Double Speaker", AutonomousModeChoices6237MR.BLUE_CENTER_DOUBLE_SPEAKER.toString());
    chooserMenu.addOption("Blue Right Auto Mode 1", AutonomousModeChoices6237MR.BLUE_RIGHT_AUTO_MODE_1.toString());
    chooserMenu.addOption("Blue Stay Clear but leave start", AutonomousModeChoices6237MR.BLUE_STAY_CLEAR.toString());
    
    chooserMenu.addOption("Red Left Auto Mode 1", AutonomousModeChoices6237MR.RED_LEFT_AUTO_MODE_1.toString()); //inverted Blue Right Auto
    chooserMenu.addOption("Red Center Double Speaker", AutonomousModeChoices6237MR.RED_CENTER_DOUBLE_SPEAKER.toString());
    chooserMenu.addOption("Red Right Auto Mode 1", AutonomousModeChoices6237MR.RED_RIGHT_AUTO_MODE_1.toString()); //inverted Blue Left Auto
    chooserMenu.addOption("Red Stay Clear but leave start", AutonomousModeChoices6237MR.RED_STAY_CLEAR.toString());

    chooserMenu.addOption("DEBUG Arm", AutonomousModeChoices6237MR.DEBUG_ARM_AUTO.toString());
    chooserMenu.addOption("DEBUG Intake", AutonomousModeChoices6237MR.DEBUG_INTAKE_AUTO.toString());
    chooserMenu.addOption("DEBUG Movement", AutonomousModeChoices6237MR.DEBUG_MOVEMENT.toString());
    chooserMenu.addOption("DEBUG Robot Rotation Command", AutonomousModeChoices6237MR.DEBUG_ROBOT_ROTATION.toString());
    chooserMenu.addOption("DEBUG Feed Intake and Fire Launcher", AutonomousModeChoices6237MR.DEBUG_FEED_INTAKE_FIRE_LAUNCER.toString());
    chooserMenu.addOption("DEBUG Move and Fire Launcher", AutonomousModeChoices6237MR.DEBUG_MOVE_AND_FIRE.toString());

    chooserMenu.addOption("DEBUG Fire Launcher Only", AutonomousModeChoices6237MR.FIRE_LAUNCHER_ONLY.toString());
    //one of a few different options that was used during testing. Probably should be deleted later, but leaving at this moment...
    // chooserMenu.addOption("Angle Playground", AutonomousModeChoices6237MR.ANGLE_PLAYGROUND.toString()); 

    chooserMenu.setDefaultOption("DEBUG Intake", AutonomousModeChoices6237MR.DEBUG_INTAKE_AUTO.toString());
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
