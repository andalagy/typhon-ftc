package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlidePreset;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.DetectedMotif;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.BackdropTarget;

/**
 * Right-side auto that uses trajectory building + odometry for multi-segment scoring.
 * Includes a simple one-cycle branch and mirrored variant for the opposite wall.
 */
@Autonomous(name = "Decode Auto Right", group = "Main")
public class DecodeAuto_Right extends LinearOpMode {
    protected boolean isMirrored() {
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        SlideSubsystem slides = new SlideSubsystem(hardwareMap);
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);

        gate.close();
        vision.start();

        boolean cycleRequested = false;
        DetectedMotif detectedMotif = DetectedMotif.MOTIF_B;
        while (!isStarted() && !isStopRequested()) {
            detectedMotif = vision.getCurrentMotif();
            if (gamepad1.a) {
                cycleRequested = true;
            }
            telemetry.addData("Detected Motif", detectedMotif);
            telemetry.addData("Cycle after preload?", cycleRequested);
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

        // 1-3. leave the launch line, slide over, and ease into the backdrop lane
        slides.goToPreset(SlidePreset.HIGH);
        Trajectory preloadPath = new TrajectoryBuilder(startPose)
                .lineTo(new Pose2d(0, 22, 0), 0.65)
                .lineTo(new Pose2d(10 * mirror, 22, 0), 0.6)
                .lineTo(new Pose2d(10 * mirror, 30, 0), 0.45)
                .build();
        drive.followTrajectory(preloadPath, this);
        while (opModeIsActive() && !slides.isAtTarget()) {
            drive.strafeWithHeading(10 * mirror, 0.55, 0, this);
            while (opModeIsActive() && !slides.isAtTarget() && !slides.isFaulted()) {
                telemetry.addData("Step", "Raising slides");
                slides.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            if (slides.isFaulted()) {
                telemetry.addData("Slide fault", slides.getFaultReason());
                telemetry.update();
                return;
            }
        }

        // 3. bump into scoring range and dump the preload
        drive.driveStraightWithHeading(8, 0.35, 0, this);

        BackdropTarget target = vision.getBackdropTarget(detectedMotif);
        if (target != null) {
            double strafe = Math.max(-10, Math.min(10, target.getLateralInches() * mirror));
            drive.strafeWithHeading(strafe, 0.35, 0, this);
        }
        gate.open();
        sleep(600);
        gate.close();

        // 4. optional quick cycle: dip to the stack, grab, and re-score at LOW
        if (cycleRequested) {
            slides.goToPreset(SlidePreset.INTAKE);
            Trajectory toStack = new TrajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Pose2d(drive.getPoseEstimate().x, drive.getPoseEstimate().y - 12, 0), 0.55)
                    .lineTo(new Pose2d(drive.getPoseEstimate().x - (8 * mirror), drive.getPoseEstimate().y - 12, 0), 0.55)
                    .lineTo(new Pose2d(drive.getPoseEstimate().x - (8 * mirror), drive.getPoseEstimate().y - 2, 0), 0.4)
                    .build();
            drive.followTrajectory(toStack, this);
            intake.intakeIn();
            sleep(500);
            Trajectory backOut = new TrajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Pose2d(drive.getPoseEstimate().x, drive.getPoseEstimate().y - 10, 0), 0.45)
                    .build();
            drive.followTrajectory(backOut, this);
            intake.stop();
            Trajectory reAlign = new TrajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(drive.getPoseEstimate().x + (6 * mirror), drive.getPoseEstimate().y, 0.55)
                    .build();
            drive.followTrajectory(reAlign, this);
            slides.goToPreset(SlidePreset.LOW);
            while (opModeIsActive() && !slides.isAtTarget() && !slides.isFaulted()) {
                telemetry.addData("Step", "Cycling to LOW");
                slides.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            Trajectory reScore = new TrajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Pose2d(drive.getPoseEstimate().x, drive.getPoseEstimate().y + 6, 0), 0.4)
                    .build();
            drive.followTrajectory(reScore, this);

            if (slides.isFaulted()) {
                telemetry.addData("Slide fault", slides.getFaultReason());
                telemetry.update();
                return;
            }

            drive.driveStraightWithHeading(6, 0.35, 0, this);
            gate.open();
            sleep(450);
            gate.close();
        }

        // 5. retract and park according to the detected motif
        slides.goToPreset(SlidePreset.INTAKE);
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
