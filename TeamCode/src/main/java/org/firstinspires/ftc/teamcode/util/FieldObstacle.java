package org.firstinspires.ftc.teamcode.util;

import java.security.acl.LastOwnerException;


public class FieldObstacle {
    private double xpos;
    private double ypos;

    public FieldObstacle(double xpos, double ypos) {
        this.xpos = xpos;
        this.ypos = ypos;
    }


    /**
     * Compute the distance from a given point to this obstacle.
     *
     * @param x The x position of the field point.
     * @param y The y position of the field point.
     * @return distance
     */
    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x-xpos,2)+Math.pow(y-ypos,2));
    }

    /* Use a dot product of the unit - vector to compute the speed towards the pole.
     */

    /**
     * Compute the distance and speed to this obstacle.
     *
     * @param x Robot x position on the field
     * @param y Robot y position on the field
     * @param xRate Robot rate of x position change (x speed)
     * @param yRate Robot rate of y position change (y speed)
     * @return [distance, speed towards obstacle]
     */
    public double[] speedTowardsObstacle(double x, double y, double xRate,double yRate){
        // x and y are the robot positions. FieldObstacle.distance(x,y) computes the distance from
        // a point on the field (x,y) to this field obstacle.
        double distance = distance(x, y);
        // rhat is a unit vector with x and y components.
        // A Unit vector means:
        //      rhatx^2 + rhatx^2 = 1^2
        //
        // This means they're kind of like the opposite and adjacent sides of a triangle whose
        // hypotenuse is exactly 1.0.  You could also think of them as x and y coordinates of a
        // vector that points to the obstacle, but the vector length is 1.0
        double rhatx = (x - xpos) / distance;
        double rhaty = (y - ypos) / distance;

        // (rhatx * xRate + rhaty * yRate) is the projection of the Rate (robot velocity) towards
        // the FieldObstacle. This is called the "dot product" of two vectors. (rhat and Rate)
        double[] retval = {distance, xRate * rhatx + yRate * rhaty};
        return retval;
    }

}
