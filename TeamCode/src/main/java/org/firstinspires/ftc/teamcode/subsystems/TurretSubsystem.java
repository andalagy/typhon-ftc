package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Turret/goal alignment helper that uses vision offsets to steer the turret servo,
 * runs a shooter flywheel, and handles non-blocking feed timing.
 *
 * <p>TeleOp usage snippet (not a full OpMode):</p>
 * <pre>
 * VisionSubsystem vision = new VisionSubsystem(hardwareMap, telemetry);
 * vision.start();
 * vision.useAprilTags();
 *
 * TurretSubsystem turret = new TurretSubsystem(hardwareMap, telemetry);
 *
 * while (opModeIsActive()) {
 *     if (gamepad1.left_bumper) {
 *         turret.spinUp();
 *     } else {
 *         turret.stopShooter();
 *     }
 *
 *     if (gamepad1.a) {
 *         turret.requestFire();
 *     }
 *
 *     VisionSubsystem.BackdropTarget target = vision.getBackdropTarget(vision.getCurrentMotif());
 *     if (target != null) {
 *         turret.setVisionAim(target.headingErrorRad, target.xPixelError, true);
 *     } else {
 *         turret.setVisionAim(0.0, 0.0, false);
 *     }
 *     turret.update();
 *     telemetry.addData("Turret Ready", turret.isReady());
 *     telemetry.addData("Aim Error", turret.getAimError());
 *     telemetry.update();
 * }
 * </pre>
 */
public class TurretSubsystem {
    public static double TURRET_MIN = 0.08;
    public static double TURRET_MAX = 0.92;
    public static double TURRET_CENTER = 0.50;

    public static double kHeadingAim = 0.07; // servo units per radian
    public static double kPixelAim = 0.0015; // servo units per pixel
    public static double MAX_TURRET_STEP_PER_UPDATE = 0.02; // servo units
    public static double HEADING_DEADBAND_RAD = Math.toRadians(1.0);
    public static double PIXEL_DEADBAND = 4.0;
    public static double HEADING_READY_TOLERANCE_RAD = Math.toRadians(2.0);
    public static double PIXEL_READY_TOLERANCE = 6.0;

    public static double TARGET_RPM = 3000.0;
    public static double RPM_TOLERANCE = 75.0;
    public static double kP = 20.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0;

    public static double FEEDER_PUSH_POS = 0.7;
    public static double FEEDER_RETRACT_POS = 0.2;
    public static long PUSH_MS = 250;

    private final Servo turretServo;
    private final DigitalChannel ledGreen;
    private final DigitalChannel ledRed;
    private final DcMotorEx shooterMotor;
    private final Servo feederServo;
    private final Telemetry telemetry;
    private final ElapsedTime fireTimer = new ElapsedTime();

    private double turretPos = TURRET_CENTER;
    private double aimError = 0.0;
    private boolean aligned = false;
    private boolean ready = false;
    private boolean pendingFire = false;
    private boolean firing = false;
    private double targetRpm = 0.0;
    private double lastKP = Double.NaN;
    private double lastKI = Double.NaN;
    private double lastKD = Double.NaN;
    private double lastKF = Double.NaN;

    private boolean hasVisionTarget = false;
    private double headingErrorRad = 0.0;
    private double xPixelError = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretServo = getHardware(hardwareMap, Servo.class, RobotConstants.TURRET_SERVO_NAME);
        ledGreen = getHardware(hardwareMap, DigitalChannel.class, RobotConstants.LED_GREEN_NAME);
        ledRed = getHardware(hardwareMap, DigitalChannel.class, RobotConstants.LED_RED_NAME);
        shooterMotor = getHardware(hardwareMap, DcMotorEx.class, RobotConstants.SHOOTER_MOTOR_NAME);
        feederServo = getHardware(hardwareMap, Servo.class, RobotConstants.FEEDER_SERVO_NAME);

        ledGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRed.setMode(DigitalChannel.Mode.OUTPUT);

        turretPos = Range.clip(TURRET_CENTER, TURRET_MIN, TURRET_MAX);
        turretServo.setPosition(turretPos);

        feederServo.setPosition(Range.clip(FEEDER_RETRACT_POS, 0.0, 1.0));

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyVelocityPidf();
    }

    public void setVisionAim(double headingErrorRad, double xPixelError, boolean hasTarget) {
        this.headingErrorRad = headingErrorRad;
        this.xPixelError = xPixelError;
        this.hasVisionTarget = hasTarget;
    }

    public void update() {
        applyVelocityPidf();

        if (!hasVisionTarget) {
            aimError = 0.0;
            aligned = false;
        } else {
            boolean headingValid = Double.isFinite(headingErrorRad);
            if (headingValid && Math.abs(headingErrorRad) > HEADING_DEADBAND_RAD) {
                aimError = headingErrorRad;
                double delta = headingErrorRad * kHeadingAim;
                delta = Range.clip(delta, -MAX_TURRET_STEP_PER_UPDATE, MAX_TURRET_STEP_PER_UPDATE);
                turretPos += delta;
            } else if (Math.abs(xPixelError) > PIXEL_DEADBAND) {
                aimError = xPixelError;
                double delta = xPixelError * kPixelAim;
                delta = Range.clip(delta, -MAX_TURRET_STEP_PER_UPDATE, MAX_TURRET_STEP_PER_UPDATE);
                turretPos += delta;
            } else {
                aimError = 0.0;
            }
            turretPos = Range.clip(turretPos, TURRET_MIN, TURRET_MAX);
            turretServo.setPosition(turretPos);
            aligned = headingValid
                    ? Math.abs(headingErrorRad) < HEADING_READY_TOLERANCE_RAD
                    : Math.abs(xPixelError) < PIXEL_READY_TOLERANCE;
        }

        ready = hasVisionTarget && aligned && atSpeed();
        setLed(ready, !ready);

        if (pendingFire && !firing && ready) {
            startFiring();
        }

        if (firing) {
            if (fireTimer.milliseconds() >= PUSH_MS) {
                feederServo.setPosition(Range.clip(FEEDER_RETRACT_POS, 0.0, 1.0));
                firing = false;
            }
        }
    }

    public void requestFire() {
        if (firing || pendingFire) {
            return;
        }
        pendingFire = true;
    }

    public void spinUp() {
        setTargetRpm(TARGET_RPM);
    }

    public void stopShooter() {
        setTargetRpm(0.0);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm);
        double ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();
        double ticksPerSecond = (targetRpm / 60.0) * ticksPerRev;
        shooterMotor.setVelocity(ticksPerSecond);
    }

    public boolean atSpeed() {
        if (targetRpm <= 0.0) {
            return false;
        }
        return Math.abs(getCurrentRpm() - targetRpm) <= RPM_TOLERANCE;
    }

    public double getCurrentRpm() {
        double ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();
        return shooterMotor.getVelocity() / ticksPerRev * 60.0;
    }

    public void setTurretCenter() {
        turretPos = Range.clip(TURRET_CENTER, TURRET_MIN, TURRET_MAX);
        turretServo.setPosition(turretPos);
    }

    public boolean isAligned() {
        return aligned;
    }

    public double getAimError() {
        return aimError;
    }

    public boolean isReady() {
        return ready;
    }

    public void telemetry(Telemetry t) {
        if (t == null) {
            return;
        }
        t.addData("Turret Pos", turretPos);
        t.addData("Aim Error", aimError);
        t.addData("Aligned", aligned);
        t.addData("Ready", ready);
        t.addData("RPM", getCurrentRpm());
        t.addData("Target RPM", targetRpm);
    }

    public void setLed(boolean greenOn, boolean redOn) {
        ledGreen.setState(greenOn);
        ledRed.setState(redOn);
    }

    private void startFiring() {
        pendingFire = false;
        firing = true;
        feederServo.setPosition(Range.clip(FEEDER_PUSH_POS, 0.0, 1.0));
        fireTimer.reset();
    }

    private void applyVelocityPidf() {
        if (lastKP != kP || lastKI != kI || lastKD != kD || lastKF != kF) {
            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
            lastKF = kF;
            shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
    }

    private static <T> T getHardware(HardwareMap hardwareMap, Class<T> clazz, String name) {
        try {
            return hardwareMap.get(clazz, name);
        } catch (RuntimeException e) {
            throw new RuntimeException("Missing hardware in RobotConstants: " + name, e);
        }
    }
}
