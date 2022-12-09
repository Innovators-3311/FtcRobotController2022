package org.firstinspires.ftc.teamcode.util;

public class FieldObstacle {
    private double xpos;
    private double ypos;

    public FieldObstacle(double xpos, double ypos) {
        this.xpos = xpos;
        this.ypos = ypos;
    }

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x-xpos,2)+Math.pow(y-ypos,2));
    }

    /* Use a dot product of the unit - vector to compute the speed towards the pole.
     */
    public double[] speedTowardsObstacle(double x, double y, double xRate,double yRate){
        double distance = distance(x, y);
        double rhatx = (x - xpos) / distance;
        double rhaty = (y - ypos) / distance;
        double[] retval = {distance, xRate * rhatx + yRate * rhaty};
        return retval;
    }

}
