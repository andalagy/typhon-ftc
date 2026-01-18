package org.firstinspires.ftc.teamcode.geometry;

/**
 * Helpers for wrapping and clamping headings so we can reason about shortest-turn angles.
 */
public final class AngleUtil {
    private AngleUtil() { }

    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
