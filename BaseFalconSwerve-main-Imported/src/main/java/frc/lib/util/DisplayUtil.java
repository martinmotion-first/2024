package frc.lib.util;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class DisplayUtil {
    
    public static Trajectory offsetTrajectoryCoordinatesForDisplayByXAndY(Trajectory originalTrajectory, double xMeters, double yMeters){
        Transform2d displayAdjustment = new Transform2d(xMeters, yMeters, new Rotation2d(0));
        Trajectory adjustedTrajectory = originalTrajectory.transformBy(displayAdjustment);
        return adjustedTrajectory;
    }
}