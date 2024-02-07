package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToIntakePositionCommand6237MR extends Command {

    private ArmSubsystem m_robotArm;

    public ArmToIntakePositionCommand6237MR(ArmSubsystem theRobotArm){
        m_robotArm = theRobotArm;
    }
    
    @Override
    public void execute() {
        m_robotArm.setTargetPosition(Constants.Arm.kIntakePosition);
    }
}
