package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.geometry.AngleUtil;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;

/**
 * Lightweight mecanum pose estimator that fuses encoder deltas with an IMU heading.
 */
public class OdometryPoseEstimator {
    private Pose2d poseEstimate;
    private double lastHeading;
    private double[] lastWheelPositions;

    public OdometryPoseEstimator(Pose2d initialPose, double initialHeading, double[] initialWheelPositions) {
        this.poseEstimate = initialPose;
        this.lastHeading = initialHeading;
        this.lastWheelPositions = initialWheelPositions;
    }

    public void reset(Pose2d pose, double heading, double[] wheelPositions) {
        this.poseEstimate = pose;
        this.lastHeading = heading;
        this.lastWheelPositions = wheelPositions;
    }

    public void update(double heading, double[] wheelPositions) {
        double dFrontLeft = wheelPositions[0] - lastWheelPositions[0];
        double dFrontRight = wheelPositions[1] - lastWheelPositions[1];
        double dBackLeft = wheelPositions[2] - lastWheelPositions[2];
        double dBackRight = wheelPositions[3] - lastWheelPositions[3];

        double dXRobot = (dFrontLeft + dFrontRight + dBackLeft + dBackRight) / 4.0;
        double dYRobot = (dFrontLeft - dFrontRight - dBackLeft + dBackRight) / 4.0;

        double avgHeading = lastHeading + AngleUtil.normalizeRadians(heading - lastHeading) / 2.0;
        double cos = Math.cos(avgHeading);
        double sin = Math.sin(avgHeading);

        double dXField = dXRobot * cos - dYRobot * sin;
        double dYField = dXRobot * sin + dYRobot * cos;

        poseEstimate = new Pose2d(poseEstimate.x + dXField, poseEstimate.y + dYField, heading);

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }
}
