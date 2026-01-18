package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.OdometryPoseEstimator;
import org.firstinspires.ftc.teamcode.geometry.AngleUtil;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.segments.TrajectorySegment;

/**
 * Drives the mecanum drivetrain, tracks pose, and exposes trajectory-following helpers.
 * Field-centric drive still uses the IMU yaw, now fused into the pose estimator so the heading
 * offset knob is honored everywhere.
 */
public class DriveSubsystem {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;

    private final OdometryPoseEstimator poseEstimator;
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);

    private double headingOffset = 0; // little offset knob so we can re-zero during TeleOp
    private boolean headingHoldEnabled = false;
    private double headingHoldTargetRadians = 0;

    private boolean turnInProgress = false;
    private double turnTargetRadians = 0;

    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, RobotConstants.IMU_NAME);
        // tell the hub how it's actually bolted on so yaw numbers aren't wacky ->
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.UP, UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        poseEstimator = new OdometryPoseEstimator(poseEstimate, getHeadingRadians(), getWheelPositionsInches());
    }

    /**
     * Drive helper for field-centric mecanum math.
     * We rotate the joystick vector by the robot heading so pushing forward always heads up-field.
     * Rotation input is blended in and everything gets normalized so no wheel commands exceed ±1.
     */
    public void drive(double x, double y, double rotation, boolean slowMode) {
        updatePoseEstimate();
        double heading = getHeadingRadians();
        // rotate the joystick vector by the robot heading so controls stay field oriented
        double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

        if (headingHoldEnabled && Math.abs(rotation) < RobotConstants.HEADING_HOLD_DEADBAND) {
            rotation = Range.clip((headingHoldTargetRadians - heading) * RobotConstants.HEADING_HOLD_KP,
                    -RobotConstants.HEADING_HOLD_MAX_TURN, RobotConstants.HEADING_HOLD_MAX_TURN);
        } else {
            headingHoldTargetRadians = heading; // refresh target whenever the driver actively turns
        }

        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rotation), 1.0);
        double frontLeftPower = (rotatedY + rotatedX + rotation) / denominator;
        double backLeftPower = (rotatedY - rotatedX + rotation) / denominator;
        double frontRightPower = (rotatedY - rotatedX - rotation) / denominator;
        double backRightPower = (rotatedY + rotatedX - rotation) / denominator;

        double scale = slowMode ? RobotConstants.SLOW_SPEED : RobotConstants.NORMAL_SPEED;
        setWheelPowers(frontLeftPower * scale, frontRightPower * scale, backLeftPower * scale, backRightPower * scale);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    public double getBackRightPower() {
        return backRight.getPower();
    }

    /** sets whatever yaw we're at as the new "zero" for field-centric stuff. */
    public void resetHeading() {
        headingOffset = getRawHeadingRadians();
        headingHoldTargetRadians = 0;
        setPoseEstimate(new Pose2d(poseEstimate.x, poseEstimate.y, 0));
    }

    /** quick helper for telemetry so drivers can see field heading in degrees. */
    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    /** current heading in radians with the offset hack applied */
    public double getHeadingRadians() {
        return getRawHeadingRadians() - headingOffset;
    }

    /** raw yaw from the IMU, in radians. */
    private double getRawHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(RevHubOrientationOnRobot.AngleUnit.RADIANS);
    }

    public void enableHeadingHold(boolean enabled) {
        headingHoldEnabled = enabled;
        headingHoldTargetRadians = getHeadingRadians();
    }

    public boolean isHeadingHoldEnabled() {
        return headingHoldEnabled;
    }

    public void followTrajectory(Trajectory trajectory, LinearOpMode opMode) {
        setPoseEstimate(trajectory.getStartPose());
        for (TrajectorySegment segment : trajectory.getSegments()) {
            segment.follow(this, opMode);
        }
        stop();
    }

    public void driveToPose(Pose2d targetPose, double maxPower, LinearOpMode opMode) {
        while (opMode.opModeIsActive()) {
            updatePoseEstimate();
            Pose2d current = getPoseEstimate();

            double dx = targetPose.x - current.x;
            double dy = targetPose.y - current.y;
            double distance = Math.hypot(dx, dy);
            double headingError = AngleUtil.normalizeRadians(targetPose.heading - current.heading);

            if (distance < RobotConstants.TRAJECTORY_POSITION_TOLERANCE_IN
                    && Math.abs(headingError) < RobotConstants.TRAJECTORY_HEADING_TOLERANCE_RAD) {
                break;
            }

            double robotX = dx * Math.cos(-current.heading) - dy * Math.sin(-current.heading);
            double robotY = dx * Math.sin(-current.heading) + dy * Math.cos(-current.heading);

            double xCommand = Range.clip(robotX * RobotConstants.TRAJECTORY_KP_TRANSLATION, -maxPower, maxPower);
            double yCommand = Range.clip(robotY * RobotConstants.TRAJECTORY_KP_TRANSLATION, -maxPower, maxPower);
            double turn = Range.clip(headingError * RobotConstants.TRAJECTORY_KP_HEADING,
                    -maxPower, maxPower);

            double denominator = Math.max(Math.abs(yCommand) + Math.abs(xCommand) + Math.abs(turn), 1.0);
            setWheelPowers((yCommand + xCommand + turn) / denominator * maxPower,
                    (yCommand - xCommand - turn) / denominator * maxPower,
                    (yCommand - xCommand + turn) / denominator * maxPower,
                    (yCommand + xCommand - turn) / denominator * maxPower);
            opMode.idle();
        }
        stop();
        headingHoldTargetRadians = getHeadingRadians();
    }

    public void turnAsync(double targetHeadingDeg) {
        turnTargetRadians = Math.toRadians(targetHeadingDeg);
        turnInProgress = true;
    }

    public boolean isTurnInProgress() {
        return turnInProgress;
    }

    public void updateAsync(LinearOpMode opMode) {
        updatePoseEstimate();
        if (turnInProgress && opMode.opModeIsActive()) {
            double headingError = AngleUtil.normalizeRadians(turnTargetRadians - getHeadingRadians());
            if (Math.abs(headingError) < RobotConstants.TRAJECTORY_HEADING_TOLERANCE_RAD) {
                stop();
                turnInProgress = false;
                headingHoldTargetRadians = getHeadingRadians();
                return;
            }
            double turn = Range.clip(headingError * RobotConstants.TRAJECTORY_KP_HEADING,
                    -RobotConstants.NORMAL_SPEED, RobotConstants.NORMAL_SPEED);
            setWheelPowers(turn, -turn, turn, -turn);
        }
    }

    public void setPoseEstimate(Pose2d pose) {
        poseEstimate = pose;
        poseEstimator.reset(pose, getHeadingRadians(), getWheelPositionsInches());
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void updatePoseEstimate() {
        poseEstimator.update(getHeadingRadians(), getWheelPositionsInches());
        poseEstimate = poseEstimator.getPoseEstimate();
    }

    /** reset all four drive encoders and prepare for encoder-based motion */
    public void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** encoder + IMU helper to drive forward/backward and hold a heading */
    public void driveStraightWithHeading(double inches, double speed, double holdHeadingDeg, LinearOpMode opMode) {
        resetDriveEncoders();
        int targetTicks = ticksFromInches(Math.abs(inches));
        double direction = Math.signum(inches);
        while (opMode.opModeIsActive() && Math.abs(getAverageEncoderPosition()) < targetTicks) {
            double headingError = Math.toRadians(holdHeadingDeg) - getHeadingRadians();
            double correction = Range.clip(headingError * RobotConstants.HEADING_HOLD_KP,
                    -RobotConstants.HEADING_HOLD_MAX_TURN, RobotConstants.HEADING_HOLD_MAX_TURN);

            setWheelPowers((direction * speed) + correction,
                    (direction * speed) - correction,
                    (direction * speed) + correction,
                    (direction * speed) - correction);
            opMode.idle();
        }
        stop();
    }

    /** encoder + IMU helper to strafe while holding heading */
    public void strafeWithHeading(double inches, double speed, double holdHeadingDeg, LinearOpMode opMode) {
        resetDriveEncoders();
        int targetTicks = ticksFromInches(Math.abs(inches));
        double direction = Math.signum(inches); // right is positive
        while (opMode.opModeIsActive() && Math.abs(getAverageEncoderPosition()) < targetTicks) {
            double headingError = Math.toRadians(holdHeadingDeg) - getHeadingRadians();
            double correction = Range.clip(headingError * RobotConstants.HEADING_HOLD_KP,
                    -RobotConstants.HEADING_HOLD_MAX_TURN, RobotConstants.HEADING_HOLD_MAX_TURN);

            double base = direction * speed;
            setWheelPowers(base + correction,
                    -base - correction,
                    -base + correction,
                    base - correction);
            opMode.idle();
        }
        stop();
    }

    /** IMU-only turn helper with simple proportional control */
    public void turnToHeading(double targetHeadingDeg, double maxPower, LinearOpMode opMode) {
        double targetRad = Math.toRadians(targetHeadingDeg);
        while (opMode.opModeIsActive()) {
            double error = targetRad - getHeadingRadians();
            if (Math.abs(error) < Math.toRadians(1.5)) {
                break;
            }
            double turn = Range.clip(error * RobotConstants.HEADING_HOLD_KP * 1.5,
                    -maxPower, maxPower);
            setWheelPowers(turn, -turn, turn, -turn); // rotate in place
            opMode.idle();
        }
        stop();
        headingHoldTargetRadians = getHeadingRadians();
    }

    private void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private int ticksFromInches(double inches) {
        return (int) Math.round(inches * RobotConstants.DRIVE_TICKS_PER_INCH);
    }

    private double ticksToInches(int ticks) {
        return ticks / RobotConstants.DRIVE_TICKS_PER_INCH;
    }

    private double[] getWheelPositionsInches() {
        return new double[]{
                ticksToInches(frontLeft.getCurrentPosition()),
                ticksToInches(frontRight.getCurrentPosition()),
                ticksToInches(backLeft.getCurrentPosition()),
                ticksToInches(backRight.getCurrentPosition())
        };
    }

    private double getAverageEncoderPosition() {
        return (Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition())
                + Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition())) / 4.0;
    }
}
