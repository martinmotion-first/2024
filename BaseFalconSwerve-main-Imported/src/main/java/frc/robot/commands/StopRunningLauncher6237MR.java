package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DisplayUtil;
import frc.robot.subsystems.LauncherSubsystem;

public class StopRunningLauncher6237MR extends Command {

    private final LauncherSubsystem m_launcher;

    public StopRunningLauncher6237MR(LauncherSubsystem launcher) {
        m_launcher = launcher;
        addRequirements(m_launcher);

        DisplayUtil.log(this.getName(), "Ending constructor");
    }

    @Override
    public void execute(){
        DisplayUtil.log(this.getName(), "In execute");
        m_launcher.stopLauncher();
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
