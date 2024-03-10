package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DisplayUtil;
import frc.robot.subsystems.LauncherSubsystem;

public class RunLauncherCommand6237MR extends Command {

    private final LauncherSubsystem m_launcher;

    public RunLauncherCommand6237MR(LauncherSubsystem launcher) {
        m_launcher = launcher;
        addRequirements(m_launcher);
    }

    @Override
    public void execute(){
        DisplayUtil.log(this.getName(), "In execute");
        m_launcher.runLauncher(false);
    }

    @Override
    public void initialize() {
        super.initialize();
        DisplayUtil.log(this.getName(), "In initialize");
    }

    @Override
    public boolean isFinished(){
        DisplayUtil.log(this.getName(), "In isFinished");
        return super.isFinished(); //or false for the moment?
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        DisplayUtil.log(this.getName(), "In end with interrupted variable: " + interrupted + "<<<<<");
    }
}
