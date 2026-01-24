package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.AprilTagMeaning;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.BackdropTarget;

/**
 * Non-blocking autonomous state machine for the DECODE game.
 * Uses the existing Drive/Intake/Turret/Vision subsystems and a timer-based flow.
 */
public class DecodeAutoRoutine {
    private static final double TARGET_RANGE_INCHES = 22.0; // TODO: tune
    private static final double RANGE_TOLERANCE_INCHES = 1.5; // TODO: tune
    private static final double DRIVE_MAX_POWER = 0.55; // TODO: tune
    private static final double STRAFE_MAX_POWER = 0.45; // TODO: tune
    private static final double TURN_MAX_POWER = 0.35; // TODO: tune

    private static final double RANGE_KP = 0.03; // TODO: tune
    private static final double LATERAL_KP = 0.04; // TODO: tune
    private static final double YAW_KP = 0.6; // TODO: tune

    private static final double SPIN_UP_TIME_SEC = 1.0; // TODO: tune
    private static final double APPROACH_FALLBACK_TIME_SEC = 1.2; // TODO: tune
    private static final double INTAKE_TIME_SEC = 1.5; // TODO: tune
    private static final double RETURN_TIME_SEC = 1.4; // TODO: tune
    private static final double SCORE_TIME_SEC = 2.0; // TODO: tune
    private static final double PARK_TIME_SEC = 1.2; // TODO: tune

    private final LinearOpMode opMode;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    private final StartPos startPos;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private AutoState state = AutoState.IDLE;
    private boolean fireRequested = false;

    private enum AutoState {
        IDLE,
        SPIN_UP,
        ALIGN_APPROACH,
        INTAKE,
        RETURN,
        SCORE,
        PARK,
        DONE
    }

    public DecodeAutoRoutine(LinearOpMode opMode, StartPos startPos) {
        this.opMode = opMode;
        this.startPos = startPos;
        drive = new DriveSubsystem(opMode.hardwareMap);
        intake = new IntakeSubsystem(opMode.hardwareMap);
        turret = new TurretSubsystem(opMode.hardwareMap);
        vision = new VisionSubsystem(opMode.hardwareMap, opMode.telemetry);
    }

    public void init() {
        vision.start();
        vision.useAprilTags();
        vision.applyCameraControls();
    }

    public void initLoop() {
        vision.applyCameraControls();
        AprilTagMeaning meaning = vision.getLatestAprilTagMeaning();
        BackdropTarget target = vision.getBackdropTarget(vision.getCurrentMotif());

        opMode.telemetry.addLine("DECODE Auto Init");
        opMode.telemetry.addData("Start Pos", startPos);
        opMode.telemetry.addData("Camera", vision.getCameraStatus());
        opMode.telemetry.addData("Latest Tag", meaning.getDescription());
        if (target != null) {
            opMode.telemetry.addData("Tag ID", target.tagId);
            opMode.telemetry.addData("Heading Error (rad)", target.headingErrorRad);
            opMode.telemetry.addData("X Pixel Error", target.xPixelError);
            opMode.telemetry.addData("Range (in)", target.getRangeInches());
            opMode.telemetry.addData("Lateral (in)", target.getLateralInches());
        } else {
            opMode.telemetry.addLine("Tag Pose: not visible");
        }
        opMode.telemetry.update();
    }

    public void start() {
        state = AutoState.SPIN_UP;
        stateTimer.reset();
        fireRequested = false;
        turret.spinUp();
    }

    public void update() {
        BackdropTarget target = vision.getBackdropTarget(vision.getCurrentMotif());
        boolean hasTarget = target != null;

        if (hasTarget) {
            turret.setVisionAim(target.headingErrorRad, target.xPixelError, true);
        } else {
            turret.setVisionAim(0.0, 0.0, false);
        }
        turret.update();

        switch (state) {
            case SPIN_UP:
                drive.stop();
                if (stateTimer.seconds() >= SPIN_UP_TIME_SEC) {
                    transitionTo(AutoState.ALIGN_APPROACH);
                }
                break;
            case ALIGN_APPROACH:
                if (hasTarget) {
                    double rangeError = target.getRangeInches() - TARGET_RANGE_INCHES;
                    double forward = Range.clip(rangeError * RANGE_KP, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);
                    double strafe = Range.clip(target.getLateralInches() * LATERAL_KP, -STRAFE_MAX_POWER, STRAFE_MAX_POWER);
                    double turn = Range.clip(target.headingErrorRad * YAW_KP, -TURN_MAX_POWER, TURN_MAX_POWER);
                    drive.drive(applyStartSideSign(strafe), forward, turn, false);

                    if (Math.abs(rangeError) <= RANGE_TOLERANCE_INCHES) {
                        drive.stop();
                        transitionTo(AutoState.INTAKE);
                    }
                } else {
                    if (stateTimer.seconds() < APPROACH_FALLBACK_TIME_SEC) {
                        drive.drive(0.0, 0.35, 0.0, false);
                    } else {
                        drive.stop();
                        transitionTo(AutoState.INTAKE);
                    }
                }
                break;
            case INTAKE:
                intake.intakeIn();
                if (stateTimer.seconds() < INTAKE_TIME_SEC) {
                    DriveCommand intakeDrive = getIntakeDrive();
                    drive.drive(intakeDrive.strafe, intakeDrive.forward, 0.0, false);
                } else {
                    drive.stop();
                    intake.stop();
                    transitionTo(AutoState.RETURN);
                }
                break;
            case RETURN:
                if (stateTimer.seconds() < RETURN_TIME_SEC) {
                    drive.drive(0.0, -0.4, 0.0, false);
                } else {
                    drive.stop();
                    transitionTo(AutoState.SCORE);
                }
                break;
            case SCORE:
                drive.stop();
                if (!fireRequested && hasTarget && turret.isReady()) {
                    turret.requestFire();
                    fireRequested = true;
                }
                if (stateTimer.seconds() >= SCORE_TIME_SEC) {
                    transitionTo(AutoState.PARK);
                }
                break;
            case PARK:
                if (stateTimer.seconds() < PARK_TIME_SEC) {
                    DriveCommand parkDrive = getParkDrive();
                    drive.drive(parkDrive.strafe, parkDrive.forward, 0.0, false);
                } else {
                    drive.stop();
                    turret.stopShooter();
                    transitionTo(AutoState.DONE);
                }
                break;
            case DONE:
            default:
                drive.stop();
                intake.stop();
                turret.stopShooter();
                break;
        }

        AprilTagMeaning meaning = vision.getLatestAprilTagMeaning();
        opMode.telemetry.addData("Auto State", state);
        opMode.telemetry.addData("Start Pos", startPos);
        opMode.telemetry.addData("Latest Tag", meaning.getDescription());
        if (hasTarget) {
            opMode.telemetry.addData("Tag ID", target.tagId);
            opMode.telemetry.addData("Heading Error (rad)", target.headingErrorRad);
            opMode.telemetry.addData("X Pixel Error", target.xPixelError);
            opMode.telemetry.addData("Range (in)", target.getRangeInches());
            opMode.telemetry.addData("Lateral (in)", target.getLateralInches());
        } else {
            opMode.telemetry.addLine("Tag Pose: not visible");
        }
        turret.telemetry(opMode.telemetry);
        opMode.telemetry.update();
    }

    public void stop() {
        drive.stop();
        intake.stop();
        turret.stopShooter();
        vision.stop();
    }

    private void transitionTo(AutoState newState) {
        state = newState;
        stateTimer.reset();
    }

    private DriveCommand getIntakeDrive() {
        double forward = 0.2;
        double strafe = 0.0;
        switch (startPos) {
            case SIDE_LEFT:
                strafe = -0.35;
                break;
            case SIDE_RIGHT:
                strafe = 0.35;
                break;
            case CENTER_LEFT:
                forward = 0.3;
                strafe = -0.2;
                break;
            case CENTER_RIGHT:
                forward = 0.3;
                strafe = 0.2;
                break;
            default:
                break;
        }
        return new DriveCommand(strafe, forward);
    }

    private DriveCommand getParkDrive() {
        double forward = -0.2;
        double strafe = 0.0;
        switch (startPos) {
            case SIDE_LEFT:
                strafe = -0.35;
                break;
            case SIDE_RIGHT:
                strafe = 0.35;
                break;
            case CENTER_LEFT:
                strafe = -0.2;
                break;
            case CENTER_RIGHT:
                strafe = 0.2;
                break;
            default:
                break;
        }
        return new DriveCommand(strafe, forward);
    }

    private double applyStartSideSign(double value) {
        switch (startPos) {
            case SIDE_RIGHT:
            case CENTER_RIGHT:
                return -value;
            case SIDE_LEFT:
            case CENTER_LEFT:
            default:
                return value;
        }
    }

    private static class DriveCommand {
        private final double strafe;
        private final double forward;

        private DriveCommand(double strafe, double forward) {
            this.strafe = strafe;
            this.forward = forward;
        }
    }
}
