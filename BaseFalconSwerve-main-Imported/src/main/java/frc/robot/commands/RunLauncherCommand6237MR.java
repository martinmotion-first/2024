package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class RunLauncherCommand6237MR extends Command {

    LauncherSubsystem m_launcher;

    public RunLauncherCommand6237MR(LauncherSubsystem launcher) {
        m_launcher = launcher;
        addRequirements(m_launcher);
    }

    @Override
    public void execute(){
        m_launcher.runLauncher();
    }
}
