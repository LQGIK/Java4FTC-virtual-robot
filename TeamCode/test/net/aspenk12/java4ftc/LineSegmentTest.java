package net.aspenk12.java4ftc;


import static org.junit.jupiter.api.Assertions.assertEquals;

import net.aspenk12.java4ftc.ps3.LineSegment;
import net.aspenk12.java4ftc.ps3.Point;
import org.junit.jupiter.api.Test;


public class LineSegmentTest {

    @Test
    public void testInterpolate() {
        Point a = new Point(0, 0);
        Point b = new Point(3, 4);
        LineSegment line = new LineSegment(a, b);
        Point c = line.interpolate(b,-5);
        assertEquals(a, c);
    }
}