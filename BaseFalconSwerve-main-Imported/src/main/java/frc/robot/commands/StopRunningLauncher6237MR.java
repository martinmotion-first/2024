package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class StopRunningLauncher6237MR extends Command {

    LauncherSubsystem m_launcher;

    public StopRunningLauncher6237MR(LauncherSubsystem launcher) {
        m_launcher = launcher;
    }

    @Override
    public void execute(){
        m_launcher.stopLauncher();
    }
}
