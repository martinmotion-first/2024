package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand6237MR extends Command {
    
    private final IntakeSubsystem intake;

    public StopIntakeCommand6237MR(IntakeSubsystem _intake) {
        this.intake = _intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.retract();
    }

    @Override
    public void execute() {
        intake.setPower(0.0);
    }

    // @Override
    // public void end(boolean interrupted) {
        
    // }

    @Override
    public boolean isFinished() {
        return true;
    }
}
