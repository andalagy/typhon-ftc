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

    // vision defaults
    public enum Motif {
        MOTIF_A, MOTIF_B, MOTIF_C
    }
}
