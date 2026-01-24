package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.AprilTagMeaning;

/**
 * Non-blocking autonomous state machine for the DECODE game.
 * Uses the existing Drive/Intake/Vision subsystems and a timer-based flow.
 */
public class DecodeAutoRoutine {
    private static final double INTAKE_TIME_SEC = 1.5; // TODO: tune
    private static final double RETURN_TIME_SEC = 1.4; // TODO: tune
    private static final double PARK_TIME_SEC = 1.2; // TODO: tune

    private final LinearOpMode opMode;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;

    private final StartPos startPos;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private AutoState state = AutoState.IDLE;

    private enum AutoState {
        IDLE,
        INTAKE,
        RETURN,
        PARK,
        DONE
    }

    public DecodeAutoRoutine(LinearOpMode opMode, StartPos startPos) {
        this.opMode = opMode;
        this.startPos = startPos;
        drive = new DriveSubsystem(opMode.hardwareMap);
        intake = new IntakeSubsystem(opMode.hardwareMap);
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

        opMode.telemetry.addLine("DECODE Auto Init");
        opMode.telemetry.addData("Start Pos", startPos);
        opMode.telemetry.addData("Camera", vision.getCameraStatus());
        opMode.telemetry.addData("Latest Tag", meaning.getDescription());
        opMode.telemetry.update();
    }

    public void start() {
        state = AutoState.INTAKE;
        stateTimer.reset();
    }

    public void update() {
        switch (state) {
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
                    transitionTo(AutoState.PARK);
                }
                break;
            case PARK:
                if (stateTimer.seconds() < PARK_TIME_SEC) {
                    DriveCommand parkDrive = getParkDrive();
                    drive.drive(parkDrive.strafe, parkDrive.forward, 0.0, false);
                } else {
                    drive.stop();
                    transitionTo(AutoState.DONE);
                }
                break;
            case DONE:
            default:
                drive.stop();
                intake.stop();
                break;
        }

        AprilTagMeaning meaning = vision.getLatestAprilTagMeaning();
        opMode.telemetry.addData("Auto State", state);
        opMode.telemetry.addData("Start Pos", startPos);
        opMode.telemetry.addData("Latest Tag", meaning.getDescription());
        opMode.telemetry.update();
    }

    public void stop() {
        drive.stop();
        intake.stop();
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

    private static class DriveCommand {
        private final double strafe;
        private final double forward;

        private DriveCommand(double strafe, double forward) {
            this.strafe = strafe;
            this.forward = forward;
        }
    }
}
