package org.firstinspires.ftc.teamcode.scarlet.utils;

public class Pose {
    public double x;
    public double y;
    public double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose() {
        this(0, 0, 0);
    }

    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public String toString() {
        return String.format("Pose(x: %.2f, y: %.2f, heading: %.2f)", x, y, Math.toDegrees(heading));
    }
}