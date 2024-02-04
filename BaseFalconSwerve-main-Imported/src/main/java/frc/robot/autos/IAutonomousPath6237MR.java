package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;

public interface IAutonomousPath6237MR {
    public List<Trajectory> getTrajectoryList();
    public double getSimulatorDisplayCoordinateX();
    public double getSimulatorDisplayCoordinateY();
}
