import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Robot;

class RobotTest {

	static Robot robot;
	
	@BeforeAll
	static void setUpBeforeClass() throws Exception {
		robot = new Robot();
	}

	@Test
	void testRobotInit() {
		//setup

		//Act
		robot.robotInit();
		
		//Assert
		assertNotNull(robot.ctreConfigs); // <-- This _should_ be accessed statically to be used like this (via Robot.ctreConfigs), but...
										  // ...testing the instantiation requires it this way...
	}

}
