package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectory.segments.HolonomicMoveSegment;
import org.firstinspires.ftc.teamcode.trajectory.segments.TrajectorySegment;

import java.util.ArrayList;
import java.util.List;

/**
 * Fluent builder for stringing together simple holonomic trajectory segments.
 */
public class TrajectoryBuilder {
    private final List<TrajectorySegment> segments = new ArrayList<>();
    private final Pose2d startPose;
    private Pose2d currentPose;

    public TrajectoryBuilder(Pose2d startPose) {
        this.startPose = startPose;
        this.currentPose = startPose;
    }

    public TrajectoryBuilder lineTo(Pose2d targetPose, double maxPower) {
        segments.add(new HolonomicMoveSegment(targetPose, maxPower));
        currentPose = targetPose;
        return this;
    }

    public TrajectoryBuilder strafeTo(double x, double y, double maxPower) {
        return lineTo(new Pose2d(x, y, currentPose.heading), maxPower);
    }

    public TrajectoryBuilder splineTo(Pose2d targetPose, double midPointFraction, double maxPower) {
        double midX = currentPose.x + (targetPose.x - currentPose.x) * midPointFraction;
        double midY = currentPose.y + (targetPose.y - currentPose.y) * midPointFraction;
        lineTo(new Pose2d(midX, midY, targetPose.heading), maxPower);
        return lineTo(targetPose, maxPower);
    }

    public Trajectory build() {
        return new Trajectory(startPose, new ArrayList<>(segments));
    }
}
