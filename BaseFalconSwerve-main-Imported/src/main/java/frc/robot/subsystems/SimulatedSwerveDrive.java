package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain.SimSwerveModule;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedSwerveDrive extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public com.ctre.phoenix6.hardware.Pigeon2 gyro;
    public SimSwerveDrivetrain simSwerveDrivetrain;

    //Note from Brian - 30 inches in meters = .762 - assuming full distance for rough estimate and because not in the lab to measure
    final double DRIVE_BASE_LENGTH_IN_METERS = .762;
    final double DRIVE_BASE_WIDTH_IN_METERS = .762;

    public SimulatedSwerveDrive() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        zeroGyro();
        // mSwerveMods = new SwerveModule[] {
        //     // new SwerveModule(0, Constants.Swerve.Mod0.constants),
        //     // new SwerveModule(1, Constants.Swerve.Mod1.constants),
        //     // new SwerveModule(2, Constants.Swerve.Mod2.constants),
        //     // new SwerveModule(3, Constants.Swerve.Mod3.constants)
        //     new com.ctre.phoenix6.mechanisms.swerve.SwerveModule()

        // };

        //Note from Brian - I apologize for the compass direction designations, but it made sense in the moment

        Translation2d wheelLocationSouthWest = new Translation2d(0,0);//bottom left
        Translation2d wheelLocationSouthEast = new Translation2d(DRIVE_BASE_LENGTH_IN_METERS,0);//bottom left
        Translation2d wheelLocationNorthWest = new Translation2d(0,DRIVE_BASE_WIDTH_IN_METERS);//bottom left
        Translation2d wheelLocationNorthEast = new Translation2d(DRIVE_BASE_LENGTH_IN_METERS, DRIVE_BASE_WIDTH_IN_METERS);
        Translation2d[] wheelLocations = {wheelLocationSouthWest, wheelLocationSouthEast, wheelLocationNorthWest, wheelLocationNorthEast};
        SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants();
        SwerveModuleConstants[] moduleConstants = {new SwerveModuleConstants()};

        simSwerveDrivetrain = new SimSwerveDrivetrain(
            wheelLocations, 
            gyro, 
            drivetrainConstants, 
            moduleConstants);
        simSwerveDrivetrain.update(DRIVE_BASE_WIDTH_IN_METERS, DRIVE_BASE_LENGTH_IN_METERS, null);
        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        // SimSwerveModule swerveSim = new SimSwerveModule(
        //     0,
        //     0, 
        //     0, 
        //     false, 
        //     0, 
        //     0, 
        //     0, 
        //     false);    
        
        //This is the simulation for drivetrain offered directly out of WPI lib, but as they say - you should use the vendor simulation as possible because they 
        //dont recommend using this necessarily 
        //The specific notes: "It is not possible to simulate encoders that are directly connected to CAN motor controllers using WPILib classes. For more information about your specific motor controller, please read your vendor’s documentation."
        //"It is not possible to simulate certain vendor gyros (i.e. Pigeon IMU and NavX) using WPILib classes. Please read the respective vendors’ documentation for information on their simulation support."
        // Create the simulation model of our drivetrain.
        // DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        // DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
        // 7.29,                    // 7.29:1 gearing reduction.
        // 7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
        // 60.0,                    // The mass of the robot is 60 kg.
        // Units.inchesToMeters(3), // The robot uses 3" radius wheels.
        // 0.7112,                  // The track width is 0.7112 meters.
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
        
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    // //Note from Brian - was just playing around here in messing with sensor simulation - this is not currently used
    // public void pigeonSim(){
    //     // BasePigeonSimCollection gyroSim = gyro.getSimCollection(); 
    //     double rawHeadingIThinkProbablyInDegrees = 0.0;
    //     /*
    //     * (from https://store.ctr-electronics.com/content/api/cpp/html/classctre_1_1phoenix_1_1sensors_1_1_base_pigeon_sim_collection.html)
    //     * Sets the simulated input heading position of the Pigeon IMU.
    //     The Pigeon IMU integrates the delta between each new raw heading value and uses this to calculate the true reported yaw and fused heading.
    //     When using the WPI Sim GUI, you will notice a readonly 'yaw' and settable 'RawHeading'. The readonly signal is the emulated yaw which will match self-test in Tuner and the hardware API. Changes to 'RawHeading' will be integrated into the emulated yaw. This way a simulator can modify the heading without overriding your hardware API calls for home-ing your sensor.
    //     Inputs to this function over time should be continuous, as user calls of setYaw() or setFusedHeading() will be accounted for in the calculation.
    //     */
    //     gyroSim.setRawHeading(rawHeadingIThinkProbablyInDegrees);
    //     // double thisIsHeadingInDegrees = gyro.getAbsoluteCompassHeading();
    //     // gyroSim.addHeading(thisIsHeadingInDegrees) //
    // }

    public Rotation2d getYaw() {
        // return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
        // Rotation2d.fromDegrees(0)
        // gyro.getSimCollection().setRawHeading(90)
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}