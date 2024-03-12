package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DisplayUtil;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToCoordinatesCommand extends Command {
    private final SwerveSubsystem swerveDriveSubsystem;
    private final double targetX;
    private final double targetY;
    private final Timer timer;

    public MoveToCoordinatesCommand(SwerveSubsystem swerveDriveSubsystem, double targetX, double targetY) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetX = targetX;
        this.targetY = targetY;
        this.timer = new Timer();

        addRequirements(swerveDriveSubsystem);
        DisplayUtil.log(getName(), "In constructor");
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        DisplayUtil.log(getName(), "In initilalize");
    }

    @Override
    public void execute() {
        double currentX = swerveDriveSubsystem.swerveOdometry.getPoseMeters().getX();
        double currentY = swerveDriveSubsystem.swerveOdometry.getPoseMeters().getY();
        double distance = Math.sqrt(Math.pow(targetX - currentX, 2) + Math.pow(targetY - currentY, 2));
        double angle = Math.atan2(targetY - currentY, targetX - currentX);

        // Limit speed based on distance to target
        double speed = Math.min(Constants.AutonomousModeConstants.kMaxSpeedMetersPerSecond, distance);

        DisplayUtil.log(getName(), "In execute, currentX:" + currentX);
        DisplayUtil.log(getName(), "In execute, currentY:" + currentY);
        DisplayUtil.log(getName(), "In execute, distance:" + distance);
        DisplayUtil.log(getName(), "In execute, angle:" + angle);
        DisplayUtil.log(getName(), "In execute, speed:" + speed + " <-- In particular check this, I think at our currently reduced speed this could be an issue");

        // Drive towards target at calculated speed and angle
        // swerveDriveSubsystem.drive(speed, angle);
        swerveDriveSubsystem.driveAlternateForAutonomous(new Translation2d(targetX, targetY), angle, false, false, speed);
    }

    @Override
    public void end(boolean interrupted) {
        DisplayUtil.log(getName(), "In end, interrupted:" + interrupted);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Check if the robot is close enough to the target coordinates
        boolean areWeFinished = Math.sqrt(Math.pow(targetX - swerveDriveSubsystem.swerveOdometry.getPoseMeters().getX(), 2) + Math.pow(targetY - swerveDriveSubsystem.swerveOdometry.getPoseMeters().getY(), 2)) < 0.1 || timer.get() > 10; // Add a timeout (10 seconds in this example)
        DisplayUtil.log(getName(), "isFinished with value: " + areWeFinished);
        DisplayUtil.log(getName(), "isFinished timer at: " + timer.get());
        return areWeFinished;
    }
}