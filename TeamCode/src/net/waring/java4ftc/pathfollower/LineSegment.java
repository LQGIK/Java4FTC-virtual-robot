package net.waring.java4ftc.pathfollower;

import java.util.Arrays;

public class LineSegment {

    private Point a;
    private Point b;
    private double slope;
    private double yInt;

    public LineSegment(Point a, Point b){
        this.a = a;
        this.b = b;
        this.slope = a.calcSlope(b);
        this.yInt = a.calcYInt(b);
    }

    public String toString(){
        return "Points: " + a + ", " + b + "\nSlope: " + slope + "\nB: " + yInt;
    }

    public double[] domainRestriction(){
        /**
         * Returns lower and upper bounds of domain
         */
        double[] boundaries = {this.a.getX(), this.b.getX()};
        Arrays.sort(boundaries);
        return boundaries;
    }

    public double[] rangeRestriction(){
        double[] boundaries = {this.a.getY(), this.b.getY()};
        Arrays.sort(boundaries);
        return boundaries;
    }

    public boolean onLine(Point point){
        /**
         * @param (Point point)
         * @return Returns true if a given point is on the line segment
         */

        // Initialize coordinates and restrictions
        double x = point.getX();
        double y = point.getY();
        double[] yRest = this.rangeRestriction();
        double[] xRest = this.domainRestriction();

        // Test if point is on a vertical line
        if (xRest[0] == xRest[1] && xRest[0] == x){
            return true;
        }

        // Test if point is on a horizontal line
        else if (yRest[0] == yRest[1] && yRest[0] == y){
            return true;
        }

        // Test regular slope line
        else if ((point.getX() * slope) + yInt == point.getY()){

            // Check domain & range restriction
            if (x >= xRest[0] && x <= xRest[1] && y >= yRest[0] && y <= yRest[1]){
                return true;
            }

        }

        // If all else fails, return false
        return false;
    }

    public Point[] subDivide(int subSegments){
        /**
         * @param Number of segments to divide line into
         * @return Returns a list of points at each segment
         */
        if (subSegments <= 0){
            return null;
        }

        // Initialize return array
        Point[] points = new Point[subSegments - 1];

        // Initialize ratio
        double ratio = 1.0 / subSegments;

        // Init total X and Y distance
        double yDist = b.getY() - a.getY();
        double xDist = b.getX() - a.getX();

        // Iterate and add the deltaX to the original x for each segment
        for (int i=0; i < subSegments - 1; i++){
            double new_x = a.getX() + ((i+1) * ratio * xDist);
            double new_y = a.getY() + ((i+1) * ratio * yDist);
            Point newPoint = new Point(new_x, new_y);
            points[i] = newPoint;
        }

        return points;
    }

    public Point calcMidPoint(){
        /**
         * @param None
         * @return Returns Point midpoint
         */

        return this.subDivide(2)[0];
    }


    public Point interpolate(Point start, double distance){
        /**
         * @param (Starting Point, Distance from point)
         * @return Returns point along this line that is a given distance away.
         */

        // Assuming a or b isn't on an axis. Otherwise we have some issues with this algorithm

        // check if point is on line
        if (!onLine(start)){
            return null;
        }

        // Calculate ratio of hypotenuses because this is same ratio for X and Y
        double ratio = distance / a.distance(b);

        // Calculate total x and y distance between a and b
        double xDist = b.getX() - a.getX();
        double yDist = b.getY() - a.getY();

        // Calculate new point (If distance is negative, move right to left, opposite if otherwise)
        double new_x = distance > 0 ? a.getX() + (ratio * xDist) : b.getX() + (ratio * xDist);
        double new_y = distance > 0 ? a.getY() + (ratio * yDist) : b.getY() + (ratio * yDist);
        new_x = Math.round((new_x * 10000.0) / 10000.0);
        new_y = Math.round((new_y * 10000.0) / 10000.0);

        return new Point(new_x, new_y);

    }

}
