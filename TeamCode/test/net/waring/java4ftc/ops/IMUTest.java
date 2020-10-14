package net.waring.java4ftc.ops;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class IMUTest {

    @Test
    void updateWraps() {

        // deltaAngle starts out at 0 because we have not wrapped yet
        Double previousAngle        = 0.0;      // Not wrapped {-180 to 180}
        double currentAngle         = 0.0;      // Not wrapped {-180 to 180}
        double deltaAngle           = 0.0;      // Value to add to currentAngle to get the currentWrappedAngle

        // Simulate turning counterclockwise from -1 to 1
        previousAngle = -1.0; currentAngle = 1.0;
        deltaAngle = IMU.updateWraps(previousAngle, currentAngle, deltaAngle);
        assertEquals(currentAngle + deltaAngle, 1);

        // Simulate that we continue turning counterclockwise from 180 to -180
        previousAngle = 180.0; currentAngle = -180.0;
        deltaAngle = IMU.updateWraps(previousAngle, currentAngle, deltaAngle);
        assertEquals(currentAngle + deltaAngle, 180.0);

        // Simulate that we continue turning counterclockwise from 540 to -180
        previousAngle = 180.0; currentAngle = -180.0;
        deltaAngle = IMU.updateWraps(previousAngle, currentAngle, deltaAngle);
        assertEquals(currentAngle + deltaAngle, 540.0);
    }
}