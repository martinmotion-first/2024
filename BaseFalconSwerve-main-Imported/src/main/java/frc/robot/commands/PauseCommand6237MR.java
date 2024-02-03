package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

public class PauseCommand6237MR extends Command {
    private long m_millisecondsToDelayInPlace = 0;

    public PauseCommand6237MR(long millisecondsToDelayInPlace){
        m_millisecondsToDelayInPlace = millisecondsToDelayInPlace;
    }
    
    @Override
    public void execute() {
        try{
            Thread.sleep(m_millisecondsToDelayInPlace);
        }catch(Exception e){}
    }
}
