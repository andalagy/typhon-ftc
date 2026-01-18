package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.DetectedMotif;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.BackdropTarget;

/**
 * Right-side auto that uses trajectory building + odometry for a multi-segment approach.
 * Includes a mirrored variant for the opposite wall.
 */
@Autonomous(name = "Decode Auto Right", group = "Main")
public class DecodeAuto_Right extends LinearOpMode {
    protected boolean isMirrored() {
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);

        vision.start();

        DetectedMotif detectedMotif = DetectedMotif.MOTIF_B;
        while (!isStarted() && !isStopRequested()) {
            detectedMotif = vision.getCurrentMotif();
            telemetry.addData("Detected Motif", detectedMotif);
            telemetry.addData("Mirrored?", isMirrored());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        drive.resetHeading();
        detectedMotif = vision.getCurrentMotif();
        vision.useAprilTags();
        double mirror = isMirrored() ? -1.0 : 1.0;

        // 1-3. leave the launch line and ease into the backdrop lane
        Trajectory preloadPath = new TrajectoryBuilder(startPose)
                .lineTo(new Pose2d(0, 22, 0), 0.65)
                .lineTo(new Pose2d(10 * mirror, 22, 0), 0.6)
                .lineTo(new Pose2d(10 * mirror, 30, 0), 0.45)
                .build();
        drive.followTrajectory(preloadPath, this);

        BackdropTarget target = vision.getBackdropTarget(detectedMotif);
        if (target != null) {
            double strafe = Math.max(-10, Math.min(10, target.getLateralInches() * mirror));
            drive.strafeWithHeading(strafe, 0.35, 0, this);
        }

        // 4. park according to the detected motif
        Trajectory clearBackdrop = new TrajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Pose2d(drive.getPoseEstimate().x, drive.getPoseEstimate().y - 6, 0), 0.55)
                .build();
        drive.followTrajectory(clearBackdrop, this);
        if (detectedMotif == DetectedMotif.MOTIF_A) {
            Trajectory parkLeft = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(drive.getPoseEstimate().x - (14 * mirror), drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(parkLeft, this);
        } else if (detectedMotif == DetectedMotif.MOTIF_C) {
            Trajectory parkRight = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(drive.getPoseEstimate().x + (14 * mirror), drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(parkRight, this);
        }

        target = vision.getBackdropTarget(detectedMotif);
        telemetry.addData("Parking tag", target != null ? target.tagId : "none");
        telemetry.update();

        drive.stop();
        vision.stop();
    }
}
