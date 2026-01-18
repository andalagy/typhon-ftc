package org.firstinspires.ftc.teamcode.trajectory.segments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Represents a unit of motion inside a trajectory.
 */
public interface TrajectorySegment {
    void follow(DriveSubsystem drive, LinearOpMode opMode);

    Pose2d getTargetPose();
}
