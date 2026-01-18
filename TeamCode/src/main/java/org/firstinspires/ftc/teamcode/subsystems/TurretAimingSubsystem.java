package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
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
 * TurretAimingSubsystem turret = new TurretAimingSubsystem(hardwareMap, vision, telemetry);
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
 *     turret.update();
 *     telemetry.addData("Turret Ready", turret.isReady());
 *     telemetry.addData("Aim Error", turret.getAimError());
 *     telemetry.update();
 * }
 * </pre>
 */
@Config
public class TurretAimingSubsystem {
    public static double TURRET_MIN = 0.08;
    public static double TURRET_MAX = 0.92;
    public static double TURRET_CENTER = 0.50;

    public static double kAim = 0.0015;
    public static double PIXEL_DEADBAND = 4.0;
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
    private final VisionSubsystem vision;
    private final Telemetry telemetry;
    private final ElapsedTime fireTimer = new ElapsedTime();

    private VisionSubsystem.DetectedMotif desiredMotif = VisionSubsystem.DetectedMotif.MOTIF_A;
    private boolean useSeenTarget = true;
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

    public TurretAimingSubsystem(HardwareMap hardwareMap, VisionSubsystem vision, Telemetry telemetry) {
        this.vision = vision;
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

    public void update() {
        applyVelocityPidf();

        VisionSubsystem.DetectedMotif motif = useSeenTarget ? vision.getCurrentMotif() : desiredMotif;
        VisionSubsystem.BackdropTarget target = vision.getBackdropTarget(motif);
        if (target == null) {
            aimError = 0.0;
            aligned = false;
        } else {
            aimError = target.xPixelError;
            if (Math.abs(aimError) > PIXEL_DEADBAND) {
                turretPos += aimError * kAim;
                turretPos = Range.clip(turretPos, TURRET_MIN, TURRET_MAX);
                turretServo.setPosition(turretPos);
            }
            aligned = Math.abs(aimError) < PIXEL_READY_TOLERANCE;
        }

        ready = target != null && aligned && atSpeed();
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

    public void setDesiredMotif(VisionSubsystem.DetectedMotif motif) {
        if (motif != null) {
            desiredMotif = motif;
            useSeenTarget = false;
        }
    }

    public void useSeenTarget() {
        useSeenTarget = true;
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
