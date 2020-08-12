package net.aspenk12.java4ftc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import net.aspenk12.java4ftc.ps3.Point;
import org.junit.jupiter.api.Test;

public class PointTest {

    @Test
    public void distanceFromOrigin() {
        assertEquals(5,new Point(3,4).distanceFromOrigin(),0.00001);
        assertEquals(13,new Point(5,-12).distanceFromOrigin(),0.00001);
        assertEquals(25,new Point(-7,24).distanceFromOrigin(),0.00001);
        assertEquals(Math.sqrt(2),new Point(-1,-1).distanceFromOrigin(),0.00001);
    }

    @Test
    public void getQuadrant() {
        assertEquals("Quadrant 1",new Point(3,4).getQuadrant());
        assertEquals("Quadrant 4",new Point(5,-12).getQuadrant());
        assertEquals("Quadrant 2",new Point(-7,24).getQuadrant());
        assertEquals("Quadrant 3",new Point(-1,-1).getQuadrant());
    }
}