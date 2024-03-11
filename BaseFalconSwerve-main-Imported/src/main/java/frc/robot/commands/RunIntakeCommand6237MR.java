package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand6237MR extends Command {

    private final IntakeSubsystem intake;
    private final Timer timer;

    public RunIntakeCommand6237MR(IntakeSubsystem _intake) {
        this.intake = _intake;
        timer = new Timer();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(Constants.Intake.kIntakePower);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Add any additional logic to execute during the intake period
    }

    @Override
    public void end(boolean interrupted) {
        intake.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}