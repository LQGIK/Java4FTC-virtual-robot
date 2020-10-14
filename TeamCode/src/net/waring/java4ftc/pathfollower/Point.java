package net.waring.java4ftc.pathfollower;

import java.util.Objects;

public class Point {

    private double x;
    private double y;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point point = (Point) o;
        return Double.compare(point.x, this.x) == 0 &&
                Double.compare(point.y, this.y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    public double getX() {
        /**
         * @return Returns x coordinate
         */
        return x;
    }

    public double getY() {
        /**
         * @return Returns y coordinate
         */
        return y;
    }

    public double distance(Point b){
        /**
         * @param (Point b)
         * @return Returns euclidian distance between this and a given point 'b'
         */
        return Math.sqrt(Math.pow(this.x - b.getX(), 2) + Math.pow(this.y - b.getY(), 2));
    }

    public double distanceFromOrigin(){
        /**
         * @return Returns distance from origin
         */
        return this.distance(new Point(0, 0));
    }

    public String getQuadrant(){
        /**
         * @return Returns string of the quadrant that this point lies in
         */

        if (x > 0 && y > 0){
            return "Quadrant 1";
        }
        else if (x < 0 && y > 0){
            return "Quadrant 2";
        }
        else if (x < 0 && y < 0){
            return "Quadrant 3";
        }
        else if (x > 0 && y < 0){
            return "Quadrant 4";
        }
        else if (x==0 || y==0){
            return "On Axes";
        }
        else {
            return null;
        }
    }

    public double calcSlope(Point b){
        /**
         * @param (Point b)
         * @return Returns slope between this and a given point 'b'
         */
        double denominator = (x - b.getX());
        double numerator = (y - b.getY());
        if (denominator == 0) {
            return 0;
        }
        return numerator / denominator;
    }

    public double calcYInt(Point b){
        /**
         * @return Returns the y-intercept given points a and b
         */
        return y - x * this.calcSlope(b);
    }

    public Point closestPoint(Point[] points){
        /**
         * @param Array of points to compare to.
         * @return Closest point from the given array of points.
         */
        Point alpha = points[0];
        for (Point p : points){
            if (this.distance(alpha) > this.distance(p)){
                alpha = p;
            }
        }
        return alpha;
    }
}
