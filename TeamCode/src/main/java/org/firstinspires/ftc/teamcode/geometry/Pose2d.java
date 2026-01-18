package org.firstinspires.ftc.teamcode.geometry;

import java.util.Objects;

/**
 * Simple immutable pose container used by the lightweight trajectory utilities.
 */
public class Pose2d {
    public final double x;
    public final double y;
    public final double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose2d withHeading(double newHeading) {
        return new Pose2d(x, y, newHeading);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Pose2d)) return false;
        Pose2d pose2d = (Pose2d) o;
        return Double.compare(pose2d.x, x) == 0 && Double.compare(pose2d.y, y) == 0
                && Double.compare(pose2d.heading, heading) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, heading);
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x=%.2f, y=%.2f, heading=%.2f)", x, y, heading);
    }
}
