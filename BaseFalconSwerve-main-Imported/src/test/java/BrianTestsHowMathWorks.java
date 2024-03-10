import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class BrianTestsHowMathWorks {
    //creating some tests to remind myself how Java treats doubles 

    double originX = 0;
    double originY = 0;

    @BeforeAll
    static void setup(){

    }

    @Test
	void testWaypointCoordinates() {
		//setup
        double destinationX = 0;
        double destinationY = 7;

		//Act
		double waypointOneX = destinationX / 3;
        double waypointOneY = destinationY / 3;

        double waypointTwoX = (destinationX * 2) / 3;
        double waypointTwoY = (destinationY * 2) / 3;
		
		//Assert
        System.out.println(waypointOneX);
        System.out.println(waypointOneY);
        System.out.println(waypointTwoX);
        System.out.println(waypointTwoY);
	}
}
