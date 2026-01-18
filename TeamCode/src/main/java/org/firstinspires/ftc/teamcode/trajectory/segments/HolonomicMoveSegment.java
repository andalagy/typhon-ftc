package org.firstinspires.ftc.teamcode.trajectory.segments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Straight-line move segment that leverages the drive pose estimator for feedback.
 */
public class HolonomicMoveSegment implements TrajectorySegment {
    private final Pose2d targetPose;
    private final double maxPower;

    public HolonomicMoveSegment(Pose2d targetPose, double maxPower) {
        this.targetPose = targetPose;
        this.maxPower = maxPower;
    }

    @Override
    public void follow(DriveSubsystem drive, LinearOpMode opMode) {
        drive.driveToPose(targetPose, maxPower, opMode);
    }

    @Override
    public Pose2d getTargetPose() {
        return targetPose;
    }
}
