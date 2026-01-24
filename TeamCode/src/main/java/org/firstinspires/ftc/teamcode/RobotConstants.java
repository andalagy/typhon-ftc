package org.firstinspires.ftc.teamcode;

/**
 * Centralized constants for robot configuration and tuning.
 * Update these values to match your robot’s hardware and performance.
 */
public class RobotConstants {

    // ----------------------------------------------------------------
    // Hardware Names — must match EXACTLY your Robot Configuration names
    // ----------------------------------------------------------------
    public static final String FRONT_LEFT_NAME  = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME   = "backLeft";
    public static final String BACK_RIGHT_NAME  = "backRight";

    public static final String INTAKE_STORAGE_NAME = "intakeStorage";
    public static final String INTAKE_FEED_NAME    = "intakeFeed";
    public static final String HOOD_MOTOR_NAME     = "hoodMotor";

    public static final String IMU_NAME          = "imu";
    public static final String LIMELIGHT_NAME    = "limelight";

    // ----------------------------------------------------------------
    // Hood Limits (encoder ticks) and power (tune for your mechanism)
    // ----------------------------------------------------------------
    public static final int HOOD_MIN_TICKS = 0;
    public static final int HOOD_MAX_TICKS = 1200;
    public static final double HOOD_POWER = 0.6;
    public static final double HOOD_MAX_RUN_SEC = 1.0;

    // Limelight pipeline indices
    public static final int LIMELIGHT_PIPELINE_MOTIF = 0;
    public static final int LIMELIGHT_PIPELINE_APRILTAG = 1;
    // Limelight image geometry (tune to match your pipeline resolution).
    public static final double LIMELIGHT_IMAGE_WIDTH_PX = 640.0;
    public static final double LIMELIGHT_HORIZONTAL_FOV_DEG = 63.3;

    // ----------------------------------------------------------------
    // Driving Speed Multipliers
    // ----------------------------------------------------------------
    public static final double NORMAL_SPEED = 0.9;   // Default teleop drive speed
    public static final double SLOW_SPEED   = 0.4;   // Precision mode (hold bumper)

    // ----------------------------------------------------------------
    // Drive Train Geometry & Encoders
    // IMPORTANT: Update these to match your robot!
    // ----------------------------------------------------------------

    // Encoder ticks per motor revolution (update if using different motors)
    public static final double DRIVE_TICKS_PER_REV = 537.7; // Example: GoBILDA 312 RPM Yellow Jacket

    // Gear ratio = output (wheel) speed / motor speed
    public static final double DRIVE_GEAR_RATIO = 1.0;

    // Wheel diameter in inches (measure wheel tread-to-tread)
    public static final double DRIVE_WHEEL_DIAMETER_IN = 3.78;

    // Derived: ticks required to travel one inch
    public static final double DRIVE_TICKS_PER_INCH =
            (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO) /
                    (Math.PI * DRIVE_WHEEL_DIAMETER_IN);

    // Distance between left and right wheels on your robot (in inches)
    public static final double DRIVE_TRACK_WIDTH_IN = 13.5;

    // Distance between front and back wheels (in inches)
    public static final double DRIVE_WHEEL_BASE_IN  = 13.0;

    // ----------------------------------------------------------------
    // Trajectory & Loop Tuning Parameters (for autonomous motion)
    // ----------------------------------------------------------------

    // Proportional constant for translational movement
    public static final double TRAJECTORY_KP_TRANSLATION = 0.08;

    // Proportional constant for heading correction
    public static final double TRAJECTORY_KP_HEADING = 1.4;

    // Acceptable tolerance for position (in inches)
    public static final double TRAJECTORY_POSITION_TOLERANCE_IN = 0.75;

    // Acceptable tolerance for heading (in radians)
    public static final double TRAJECTORY_HEADING_TOLERANCE_RAD =
            Math.toRadians(2.0);

    // Safety timeout for any trajectory-following segment (seconds)
    public static final double TRAJECTORY_MAX_TIME_SEC = 6.0;

    // Heading hold assist PID (if used)
    public static final double HEADING_HOLD_KP        = 0.02;
    public static final double HEADING_HOLD_DEADBAND  = 0.05; // ignore tiny stick wiggles
    public static final double HEADING_HOLD_MAX_TURN   = 0.4;  // max correction power

    // Safety timeouts for blocking drive helpers (seconds)
    public static final double DRIVE_STRAIGHT_TIMEOUT_SEC = 4.0;
    public static final double STRAFE_TIMEOUT_SEC = 4.0;
    public static final double TURN_TIMEOUT_SEC = 3.0;

    // ----------------------------------------------------------------
    // Vision / Detection Labels
    // Use these as type-safe labels for detected objects or motifs
    // ----------------------------------------------------------------
    public enum Motif {
        MOTIF_A,
        MOTIF_B,
        MOTIF_C
    }
}
