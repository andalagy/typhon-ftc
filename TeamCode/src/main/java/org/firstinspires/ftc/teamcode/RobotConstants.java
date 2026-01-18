package org.firstinspires.ftc.teamcode;

/**
 * scribble pad for hardware names, directions, and the latest numbers ☆
 * subject to future changes.
 */
public class RobotConstants {
    // motor names as i typed them into the Robot Configuration screen ->
    public static final String FRONT_LEFT_NAME = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME = "backLeft";
    public static final String BACK_RIGHT_NAME = "backRight";

    public static final String INTAKE_NAME = "intakeMotor";
    public static final String LEFT_SLIDE_NAME = "leftSlide";
    public static final String RIGHT_SLIDE_NAME = "rightSlide";

    public static final String GATE_SERVO_NAME = "gateServo";
    public static final String IMU_NAME = "imu";
    public static final String WEBCAM_NAME = "Webcam 1";

    // drive speed scales
    public static final double NORMAL_SPEED = 0.9;   // full power scale for teleop ☆
    public static final double SLOW_SPEED = 0.4;     // precision mode when holding a bumper

    // drive encoder/geometry numbers (tune to your drivetrain)
    public static final double DRIVE_TICKS_PER_REV = 537.7; // goBILDA 312 RPM yellow jacket default
    public static final double DRIVE_GEAR_RATIO = 1.0;      // output (wheel) speed / motor speed
    public static final double DRIVE_WHEEL_DIAMETER_IN = 3.78; // 96 mm mecanum
    public static final double DRIVE_TICKS_PER_INCH =
            (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO) / (Math.PI * DRIVE_WHEEL_DIAMETER_IN);
    public static final double DRIVE_TRACK_WIDTH_IN = 13.5; // distance between left/right wheels
    public static final double DRIVE_WHEEL_BASE_IN = 13.0; // distance between front/back wheels
    public static final double TRAJECTORY_KP_TRANSLATION = 0.08;
    public static final double TRAJECTORY_KP_HEADING = 1.4;
    public static final double TRAJECTORY_POSITION_TOLERANCE_IN = 0.75;
    public static final double TRAJECTORY_HEADING_TOLERANCE_RAD = Math.toRadians(2.0);
    public static final double HEADING_HOLD_KP = 0.02;        // tune the heading hold assist
    public static final double HEADING_HOLD_DEADBAND = 0.05;  // ignore tiny stick wiggles
    public static final double HEADING_HOLD_MAX_TURN = 0.4;   // cap heading correction power

    // slide target positions (encoder ticks). tune for your robot.
    public static final int SLIDE_INTAKE = 0;
    public static final int SLIDE_LOW = 850;      // rough guess, tweak as needed
    public static final int SLIDE_HIGH = 1600;
    public static final int SLIDE_MAX = 1900;     // don't let the slides fly away

    // slide motion settings
    public static final double SLIDE_POWER = 1.0;
    public static final double SLIDE_HOLD_POWER = 0.05; // tiny holding power for manual modes :)
    public static final int SLIDE_POSITION_TOLERANCE = 25; // ticks away from the target to count as "arrived"
    public static final double SLIDE_CURRENT_LIMIT_AMPS = 8.5; // soft over-current guard for stalled slides
    public static final double SLIDE_STALL_VELOCITY_TICKS_PER_S = 25.0; // treat below this as "stopped"
    public static final long SLIDE_STALL_TIMEOUT_MS = 400; // how long we allow a stall before faulting
    public static final double SLIDE_STALL_MIN_POWER = 0.2; // ignore stall checks when hardly applying power
    public static final long SLIDE_FAULT_CLEAR_MS = 800; // cool-down time before we automatically retry
    public static final double SLIDE_RECOVERY_CURRENT_RATIO = 0.5; // fraction of the limit required before clearing a fault
    public static final int SLIDE_SLOW_ZONE_TICKS = 150; // start ramping down power this far from the ends
    public static final double SLIDE_SLOW_ZONE_SCALE = 0.5; // power multiplier inside the slow zone

    // gate servo positions (0-1). tune for your hardware.
    public static final double GATE_CLOSED = 0.15;
    public static final double GATE_OPEN = 0.65;

    // vision defaults
    public enum Motif {
        MOTIF_A, MOTIF_B, MOTIF_C
    }
}
