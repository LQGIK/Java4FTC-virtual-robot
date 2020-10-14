package net.waring.java4ftc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import net.waring.java4ftc.pathfollower.Path;
import net.waring.java4ftc.pathfollower.Point;
import org.junit.jupiter.api.Test;

public class PathTest {

    @Test
    public void testTargetPoint() {
        /**
         *
         *  |  |
         * |\/|
         * Total Distance: 22
         */
        Point[] pointsProjCheck = {new Point(-4, 3), new Point(-4, -3), new Point(0, 0), new Point(4, -3), new Point(4,3)};
        Path pathProjCheck = new Path(pointsProjCheck);
        Path.WayPoint result = pathProjCheck.targetPoint(new Point(-4, 3), 16);
        pathProjCheck.targetPoint(result.point, 6);



        Point[] points1 = {new Point(9, 9), new Point(9, 5), new Point(6, 4), new Point(1, 5)};
        Path path1 = new Path(points1);
        assertEquals(new Point(6, 4), path1.targetPoint(new Point(7, 8), 6.16227766).point);
        assertEquals(new Point(9, 6), path1.targetPoint(new Point(7, 8), 2).point);
        assertEquals(new Point(9, 9), path1.targetPoint(new Point(10, 9), 0).point);
        assertEquals(new Point(1, 5), path1.targetPoint(new Point(7, 8), 20).point);
        assertEquals(new Point(9,5), path1.targetPoint(new Point(7,10), 5).point);

        // Make some points
        Point[] points2 = {new Point(-.75, 0), new Point(3, 5), new Point(7, 5), new Point(7, 0), new Point(0,0)};
        Path path2 = new Path(points2);
        assertEquals(new Point(0,0), path2.targetPoint(new Point(4, -2), 50).point);
        assertEquals(new Point(3, 5), path2.targetPoint(new Point(4, -2), 5).point);
        assertEquals(new Point(0, 1), path2.targetPoint(new Point(4, -2), 0).point);

        Point[] points3 = {new Point(0,0), new Point(-2,-2)};
        Path path3 = new Path(points3);
        assertEquals(new Point(-2, -2), path3.targetPoint(new Point(0,0), 10).point);
        assertEquals(new Point(0, 0), path3.targetPoint(new Point(0,0), 0).point);



    }

    @Test
    public void distanceFromStart() {

        // Make some points
        Point a = new Point(-.75, 0);
        Point b = new Point(3, 5);
        Point c = new Point(7, 5);
        Point d = new Point(7,0);
        Point robot = new Point(0, 1);

        Point[] points = {a, b, c, d};
        Path path = new Path(points);

        // Tests

        //assertEquals(14, path.getWayPoints().get(0).distanceFromStart(a), 0001);
    }

    @Test
    public void distanceFromEnd() {
        // Make some points
        Point a = new Point(-.75, 0);
        Point b = new Point(3, 5);
        Point c = new Point(7, 5);
        Point d = new Point(0,0);

        Point robot = new Point(0, 1);

        Point[] points = {a, b, c, d};
        Path path = new Path(points);

        // Tests

    }

    @Test
    public void onPath() {
        // Make some points
        Point a = new Point(-.75, 0);
        Point b = new Point(3, 5);
        Point c = new Point(7, 5);
        Point d = new Point(0,0);

        Point robot = new Point(0, 1);

        Point[] points = {a, b, c, d};
        Path path = new Path(points);

        //Tests
        assertEquals(true, path.onPath(b));
        assertEquals(true, path.onPath(d));
        assertEquals(false, path.onPath(new Point(-1, -1)));
        assertEquals(true, path.onPath(new Point(2.25, 4)));
    }
}