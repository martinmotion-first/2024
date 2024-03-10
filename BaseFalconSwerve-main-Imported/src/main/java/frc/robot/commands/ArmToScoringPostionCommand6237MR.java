package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DisplayUtil;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToScoringPostionCommand6237MR extends Command {
    private final ArmSubsystem m_robotArm;

    public ArmToScoringPostionCommand6237MR(ArmSubsystem theRobotArm){
        m_robotArm = theRobotArm;
        addRequirements(m_robotArm);
        DisplayUtil.log(this.getName(), "Ending constructor");
    }
    
    @Override
    public void execute() {
        DisplayUtil.log(this.getName(), "Starting execute");
        m_robotArm.setTargetPosition(Constants.Arm.kScoringPosition);
        // m_robotArm.runAutomatic();
        DisplayUtil.log(this.getName(), "Ending execute");
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
