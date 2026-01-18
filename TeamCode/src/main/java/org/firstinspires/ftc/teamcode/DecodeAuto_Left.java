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
 * Left-side auto that drives off the line, aligns to the backdrop, and parks based on the detected motif.
 * Uses the lightweight trajectory follower instead of manual encoder while-loops.
 */
@Autonomous(name = "Decode Auto Left", group = "Main")
public class DecodeAuto_Left extends LinearOpMode {

    private DriveSubsystem drive;
    private VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        vision.start();

        // update telemetry during init so drivers see the live motif
        DetectedMotif detectedMotif = DetectedMotif.MOTIF_A;
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

        // 1-3. leave the Launch Line and settle onto the backdrop lane
        Trajectory preloadPath = new TrajectoryBuilder(startPose)
                .lineTo(new Pose2d(0, 20, 0), 0.55)
                .lineTo(new Pose2d(-8, 20, 0), 0.55)
                .lineTo(new Pose2d(-8, 28, 0), 0.4)
                .build();
        drive.followTrajectory(preloadPath, this);

        // Correct lateral offset using the backdrop tag if visible
        BackdropTarget target = vision.getBackdropTarget(detectedMotif);
        if (target != null) {
            double strafe = Math.max(-10, Math.min(10, target.getLateralInches()));
            drive.strafeWithHeading(strafe, 0.35, 0, this);
        }

        // 4. back away
        Trajectory retreat = new TrajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Pose2d(drive.getPoseEstimate().x, drive.getPoseEstimate().y - 8, 0), 0.5)
                .build();
        drive.followTrajectory(retreat, this);

        // 5. park based on motif
        if (detectedMotif == DetectedMotif.MOTIF_A) {
            Trajectory parkLeft = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(-16, drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(parkLeft, this);
        } else if (detectedMotif == DetectedMotif.MOTIF_C) {
            Trajectory parkRight = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(16, drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(parkRight, this);
        } // motif B stays put

        // confirm parking column/tag alignment
        target = vision.getBackdropTarget(detectedMotif);
        telemetry.addData("Parking tag", target != null ? target.tagId : "none");
        telemetry.update();

        drive.stop();
        vision.stop();
    }
}
