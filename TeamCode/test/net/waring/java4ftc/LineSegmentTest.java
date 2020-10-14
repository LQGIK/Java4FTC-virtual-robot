package net.waring.java4ftc;


import static org.junit.jupiter.api.Assertions.assertEquals;

import net.waring.java4ftc.pathfollower.LineSegment;
import net.waring.java4ftc.pathfollower.Point;
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