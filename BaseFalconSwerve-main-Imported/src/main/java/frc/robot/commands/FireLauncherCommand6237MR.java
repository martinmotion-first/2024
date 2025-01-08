package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DisplayUtil;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class FireLauncherCommand6237MR extends Command {
    private final LauncherSubsystem m_launcher;
    private final IntakeSubsystem m_intake;
    private Timer m_timer;

    public FireLauncherCommand6237MR(LauncherSubsystem launcher, IntakeSubsystem intake) {
        m_launcher = launcher;
        m_intake = intake;
        addRequirements(m_launcher, m_intake);

        DisplayUtil.log(this.getName(), "Ending constructor");
    }

    @Override
    public void execute(){
        DisplayUtil.log(this.getName(), "In execute");
        if(m_timer.get() > Constants.AutonomousModeConstants.kLauncherRunBeforeFiringDelay){
            m_intake.setPower(1.0);
        }
    }
    
    @Override
    public void initialize() {
        super.initialize();
        m_timer = new Timer();
        m_timer.start();
        m_launcher.runLauncher(false);
        DisplayUtil.log(this.getName(), "In initialize");
    }

    @Override
    public boolean isFinished(){
        boolean areWeFinished = m_timer.get() > Constants.Intake.kShotFeedTime;
        DisplayUtil.log(this.getName(), "In isFinished with value:" + areWeFinished);
        return areWeFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_launcher.stopLauncher();
        m_intake.setPower(0.0);
        DisplayUtil.log(this.getName(), "In end with interrupted variable: " + interrupted + "<<<<<");
    }
    
}
