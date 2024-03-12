a

public void stopModules() {
    SwerveModuleState[] stopStates = new SwerveModuleState[4]; // Assuming you have 4 modules
    for (int i = 0; i < stopStates.length; i++) {
        stopStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0)); // Zero speed and zero angle
    }
    setModuleStates(stopStates);
}