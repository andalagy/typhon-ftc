package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectory.segments.TrajectorySegment;

import java.util.Collections;
import java.util.List;

/**
 * Immutable container for a lightweight, segment-based trajectory.
 */
public class Trajectory {
    private final Pose2d startPose;
    private final List<TrajectorySegment> segments;

    public Trajectory(Pose2d startPose, List<TrajectorySegment> segments) {
        this.startPose = startPose;
        this.segments = Collections.unmodifiableList(segments);
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public List<TrajectorySegment> getSegments() {
        return segments;
    }
}
