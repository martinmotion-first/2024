package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyroCommand6237MR extends Command {
    private final SwerveSubsystem swerveDrive;
    
    public ZeroGyroCommand6237MR(SwerveSubsystem swerveSubsystem) {
        this.swerveDrive = swerveSubsystem;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.zeroGyro();
    }
}
