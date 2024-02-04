package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AnglePlaygroundAuto6237MR;
import frc.robot.autos.AutonomousModeChoices6237MR;
import frc.robot.autos.BlueCenterAuto6237MR;
import frc.robot.autos.BlueLeftAuto6237MR;
import frc.robot.autos.BlueRightAuto6237MR;
import frc.robot.autos.ExampleAutonomous;
import frc.robot.autos.RedCenterAuto6237MR;
import frc.robot.autos.RedLeftAuto6237MR;
import frc.robot.autos.RedRightAuto6237MR;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(0);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final LauncherSubsystem m_launcher = new LauncherSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
        m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

        // set the intake to stop (0 power) when no other command is running
        m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

        // configure the launcher to stop when no other command is running
        m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));
    }

    public SwerveDriveOdometry retrieveOdometry(){
        return s_Swerve.swerveOdometry;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    
        //revbot subsytem configurations
        // button to put swerve modules in an "x" configuration to hold position
    // new JoystickButton(operator, XboxController.Button.kLeftStick.value)
    //     .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // set up arm preset positions
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    new Trigger(
            () ->
                operator.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    new JoystickButton(operator, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

    // intake controls (run while button is held down, run retract command once when the button is released)
    new Trigger(
            () ->
                operator.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
        .onFalse(m_intake.retract());

    new JoystickButton(operator, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    new JoystickButton(operator, XboxController.Button.kA.value)
        .onTrue(m_intake.feedLauncher(m_launcher));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public SequentialCommandGroup getAutonomousCommand(String selectedOption) {
        switch (AutonomousModeChoices6237MR.valueOf(selectedOption)){
            case EXAMPLE_AUTO:
                return new ExampleAutonomous(s_Swerve);
            case BLUE_RIGHT_AUTO_MODE_1:
                return new BlueRightAuto6237MR(s_Swerve);
            case RED_LEFT_AUTO_MODE_1:
                return new RedLeftAuto6237MR(s_Swerve);
            case BLUE_CENTER_AUTO_MODE_1:
                return new BlueCenterAuto6237MR(s_Swerve);
            case RED_CENTER_AUTO_MODE_1:
                return new RedCenterAuto6237MR(s_Swerve);
            case BLUE_LEFT_AUTO_MODE_1:
                return new BlueLeftAuto6237MR(s_Swerve);
            case RED_RIGHT_AUTO_MODE_1:
                return new RedRightAuto6237MR(s_Swerve);
            case ANGLE_PLAYGROUND:
                return new AnglePlaygroundAuto6237MR(s_Swerve);
            default:
               return new ExampleAutonomous(s_Swerve);
        }
    }

    public Swerve getSwerve(){
        return s_Swerve;
    }
}
