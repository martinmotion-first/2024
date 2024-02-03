package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RotationOnlyCommand6237MR extends Command{

    private Swerve m_Swerve;
    private DoubleSupplier m_rotationSup; //this was borrowed from teleop, so "Supplier" might be needed?
    private double m_rotationAngle;

    //no idea what this will actually need to take yet, but this seems like a semi-ok place to start
    public RotationOnlyCommand6237MR(Swerve swerve, DoubleSupplier rotationSupplier, double rotationAngle){
        m_Swerve = swerve;
        m_rotationSup = rotationSupplier;
        m_rotationAngle = rotationAngle;
    }
    
    @Override
    public void execute() {
        // m_Swerve.getModulePositions()[0].angle = Rotation2d.fromDegrees(135.0); // no idea how to know which of thse module positions represents what wheel at this moment
        // m_Swerve.getModulePositions()[1].angle = Rotation2d.fromDegrees(45.0);  // if this should even involve the module positions and not the module states
        // m_Swerve.getModulePositions()[2].angle = Rotation2d.fromDegrees(-45.0); 
        // m_Swerve.getModulePositions()[3].angle = Rotation2d.fromDegrees(-135.0);
        
    //     leftFrontWheel.setDirection(135.0);
    //     leftBackWheel.setDirection(45.0);
    //     rightFrontWheel.setDirection(-45.0);
    //     rightBackWheel.setDirection(-135.0);

    //     leftFrontWheel.setSpeed(power);
    //     leftBackWheel.setSpeed(power);
    //     rightFrontWheel.setSpeed(power);
    //     rightBackWheel.setSpeed(power);
    }


    //I found this "inplaceTurn" from a team that did swerve and provided some very useful (additional) info as well
    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
    // public void inplaceTurn(double power)
    // {
    //     leftFrontWheel.setDirection(135.0);
    //     leftBackWheel.setDirection(45.0);
    //     rightFrontWheel.setDirection(-45.0);
    //     rightBackWheel.setDirection(-135.0);

    //     leftFrontWheel.setSpeed(power);
    //     leftBackWheel.setSpeed(power);
    //     rightFrontWheel.setSpeed(power);
    //     rightBackWheel.setSpeed(power);
    // }
}
