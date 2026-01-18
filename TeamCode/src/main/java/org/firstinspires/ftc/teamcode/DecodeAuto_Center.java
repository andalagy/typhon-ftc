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
 * Center-side auto with the same subsystems as TeleOp.
 * Drives forward to align with the backdrop, branches parking based on the detected motif.
 */
@Autonomous(name = "Decode Auto Center", group = "Main")
public class DecodeAuto_Center extends LinearOpMode {
    private DriveSubsystem drive;
    private VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        vision.start();

        DetectedMotif detectedMotif = DetectedMotif.MOTIF_B;
        while (!isStarted() && !isStopRequested()) {
            detectedMotif = vision.getCurrentMotif();
            vision.applyCameraControls();
            telemetry.addData("Detected Motif", detectedMotif);
            telemetry.addData("Vision camera", vision.getCameraStatus());
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

        // 1. drive straight to the backdrop
        Trajectory scoreBackdrop = new TrajectoryBuilder(startPose)
                .lineTo(new Pose2d(0, 26, 0), 0.7)
                .build();
        drive.followTrajectory(scoreBackdrop, this);

        // Center on the correct column using AprilTags if available
        BackdropTarget target = vision.getBackdropTarget(detectedMotif);
        if (target != null) {
            double strafe = Math.max(-10, Math.min(10, target.getLateralInches()));
            drive.strafeWithHeading(strafe, 0.35, 0, this);
        }

        // 2. park according to the motif
        Trajectory clearBackdrop = new TrajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Pose2d(0, 20, drive.getPoseEstimate().heading), 0.55)
                .build();
        drive.followTrajectory(clearBackdrop, this);
        if (detectedMotif == DetectedMotif.MOTIF_A) {
            Trajectory parkLeft = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(-12, drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(parkLeft, this);
        } else if (detectedMotif == DetectedMotif.MOTIF_C) {
            Trajectory parkRight = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(12, drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(parkRight, this);
        }

        drive.stop();
        vision.stop();
    }
}
