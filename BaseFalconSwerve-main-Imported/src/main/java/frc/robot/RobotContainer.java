package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.AnglePlaygroundAuto6237MR;
import frc.robot.autos.AutonomousModeChoices6237MR;
import frc.robot.autos.BlueCenterAuto6237MR;
import frc.robot.autos.BlueLeftAuto6237MR;
import frc.robot.autos.BlueRightAuto6237MR;
import frc.robot.autos.RedCenterAuto6237MR;
import frc.robot.autos.RedLeftAuto6237MR;
import frc.robot.autos.RedRightAuto6237MR;
import frc.robot.autos.doublespeaker.BlueCenterDoubleSpeakerAuto6237MR;
import frc.robot.autos.doublespeaker.BlueLeftDoubleSpeakerAuto6237MR;
import frc.robot.autos.doublespeaker.BlueRightDoubleSpeakerAuto6237MR;
import frc.robot.autos.doublespeaker.RedLeftDoubleSpeakerAuto6237MR;
import frc.robot.autos.doublespeaker.RedRightDoubleSpeakerAuto6237MR;
import frc.robot.autosDebug.AngleDebugAuto6237MR;
import frc.robot.autosDebug.AngleDebugAuto6237MR2;
import frc.robot.autosDebug.ArmDebugAuto6237MR;
import frc.robot.autosDebug.IntakeDebugAuto6237MR;
import frc.robot.autosDebug.IntakeToLauncherDebugAuto6237MR;
import frc.robot.autosDebug.MoveAndFireDebugAuto6237MR;
import frc.robot.autosDebug.MovementDebugAuto6237MR;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.DriverMapping6237MR;
import frc.robot.controllers.OperatorMapping6237MR;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(Constants.kXboxDriverPort);
    private final XboxController operator = new XboxController(Constants.kXboxOperatorPort);

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final LauncherSubsystem m_launcher = new LauncherSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(DriverMapping6237MR.translationAxis), 
                () -> -driver.getRawAxis(DriverMapping6237MR.strafeAxis), 
                () -> -driver.getRawAxis(DriverMapping6237MR.rotationAxis), 
                () -> DriverMapping6237MR.robotCentric.getAsBoolean(),
                () -> DriverMapping6237MR.invertFrontAndBackButton.getAsBoolean()
            )
        );

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
        OperatorMapping6237MR.mapXboxController(operator, m_intake, m_arm, m_launcher);
        DriverMapping6237MR.mapXboxController(driver, s_Swerve);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public SequentialCommandGroup getAutonomousCommand(String selectedOption) {
        switch (AutonomousModeChoices6237MR.valueOf(selectedOption)){
            case BLUE_RIGHT_AUTO_MODE_1:
                return new BlueRightAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case RED_LEFT_AUTO_MODE_1:
                return new RedLeftAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case BLUE_CENTER_AUTO_MODE_1:
                return new BlueCenterAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case RED_CENTER_AUTO_MODE_1:
                return new RedCenterAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case BLUE_LEFT_AUTO_MODE_1:
                return new BlueLeftAuto6237MR(s_Swerve, m_intake, m_arm, m_launcher);
            case RED_RIGHT_AUTO_MODE_1:
                return new RedRightAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);

            case BLUE_LEFT_DOUBLE_SPEAKER:
                return new BlueLeftDoubleSpeakerAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case RED_RIGHT_DOUBLE_SPEAKER:
                return new RedRightDoubleSpeakerAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case BLUE_CENTER_DOUBLE_SPEAKER:
                return new BlueCenterDoubleSpeakerAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case RED_CENTER_DOUBLE_SPEAKER:
                return new RedCenterAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case BLUE_RIGHT_DOUBLE_SPEAKER:
                return new BlueRightDoubleSpeakerAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case RED_LEFT_DOUBLE_SPEAKER:
                return new RedLeftDoubleSpeakerAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);

            case DEBUG_ARM_AUTO:
                return new ArmDebugAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case DEBUG_INTAKE_AUTO:
                return new IntakeDebugAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case DEBUG_MOVEMENT:
                return new MovementDebugAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case DEBUG_ANGLE_120:
                return new AngleDebugAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case DEBUG_ANGLE_NEG_120:
                return new AngleDebugAuto6237MR2(s_Swerve, m_arm, m_launcher, m_intake);
            case DEBUG_INTAKE_TO_LAUNCER:
                return new IntakeToLauncherDebugAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            case DEBUG_MOVE_AND_FIRE:
                return new MoveAndFireDebugAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);

            case ANGLE_PLAYGROUND:
                return new AnglePlaygroundAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
            default:
               return new AnglePlaygroundAuto6237MR(s_Swerve, m_arm, m_launcher, m_intake);
        }
    }

    public SwerveSubsystem getSwerve(){
        return s_Swerve;
    }
}
