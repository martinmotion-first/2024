package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToScoringPostionCommand6237MR extends Command {
    private ArmSubsystem m_robotArm;

    public ArmToScoringPostionCommand6237MR(ArmSubsystem theRobotArm){
        m_robotArm = theRobotArm;
        addRequirements(m_robotArm);
    }
    
    @Override
    public void execute() {
        m_robotArm.setTargetPosition(Constants.Arm.kScoringPosition);
        m_robotArm.runAutomatic();
    }
}
