package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.CurrentUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Manages the dual slide motors so they move together and stop at safe heights.
 * Preset helpers move to common scoring heights, while manualControl lets the driver trim with a stick.
 * Encoder checks keep the slides inside the intake-to-max window to protect the rigging.
 */
public class SlideSubsystem {
    public enum SlidePreset {
        INTAKE,
        LOW,
        HIGH,
        MAX
    }

    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private int targetPositionTicks = RobotConstants.SLIDE_INTAKE;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private boolean stallTimerRunning = false;
    private long faultTimestampMs = 0L;
    private boolean faulted = false;
    private String faultReason = "";
    private double lastCommandedPower = 0.0;

    public SlideSubsystem(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_SLIDE_NAME);
        rightSlide = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_SLIDE_NAME);

        // flip one side because the slides are mirrored in the real world ->
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(RunMode.RUN_USING_ENCODER);
    }

    public void goToPreset(SlidePreset preset) {
        switch (preset) {
            case LOW:
                moveToPosition(RobotConstants.SLIDE_LOW);
                break;
            case HIGH:
                moveToPosition(RobotConstants.SLIDE_HIGH);
                break;
            case MAX:
                moveToPosition(RobotConstants.SLIDE_MAX);
                break;
            case INTAKE:
            default:
                moveToPosition(RobotConstants.SLIDE_INTAKE);
                break;
        }
    }

    public void goToIntake() {
        goToPreset(SlidePreset.INTAKE);
    }

    public void goToLow() {
        goToPreset(SlidePreset.LOW);
    }

    public void goToHigh() {
        goToPreset(SlidePreset.HIGH);
    }

    public void goToMax() {
        goToPreset(SlidePreset.MAX);
    }

    public boolean isBusy() {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }

    /**
     * Manual override for the driver stick with simple software end stops.
     * Adds a tiny holding power when centered so the slides do not drift back down.
     */
    public void manualControl(double input) {
        attemptRecovery();
        double avgPosition = getAveragePosition();
        double requestedPower = input * RobotConstants.SLIDE_POWER;

        // lock out movement past the soft limits
        if ((input > 0 && avgPosition >= RobotConstants.SLIDE_MAX) ||
                (input < 0 && avgPosition <= RobotConstants.SLIDE_INTAKE)) {
            requestedPower = 0;
        }

        // small deadband with a gentle hold near where we left the slides
        if (Math.abs(input) < 0.05) {
            if (avgPosition > RobotConstants.SLIDE_INTAKE + 10) {
                requestedPower = RobotConstants.SLIDE_HOLD_POWER;
            } else {
                requestedPower = 0;
            }
        } else {
            targetPositionTicks = (int) avgPosition; // handoff to manual sets new hold position
        }

        leftSlide.setMode(RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(RunMode.RUN_USING_ENCODER);
        applyPower(requestedPower);
    }

    public void stop() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    public int getAveragePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    public int getLeftPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightSlide.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(getAveragePosition() - targetPositionTicks) <= RobotConstants.SLIDE_POSITION_TOLERANCE;
    }

    public int getTargetPosition() {
        return targetPositionTicks;
    }

    public double getAverageCurrent() {
        return (leftSlide.getCurrent(CurrentUnit.AMPS) + rightSlide.getCurrent(CurrentUnit.AMPS)) / 2.0;
    }

    public double getAverageVelocity() {
        return (leftSlide.getVelocity() + rightSlide.getVelocity()) / 2.0;
    }

    public boolean isFaulted() {
        return faulted;
    }

    public String getFaultReason() {
        return faultReason;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Slide target", getTargetPosition());
        telemetry.addData("Slide pos", getAveragePosition());
        telemetry.addData("Slide vel (t/s)", "%.1f", getAverageVelocity());
        telemetry.addData("Slide current (A)", "%.1f / %.1f", getAverageCurrent(), RobotConstants.SLIDE_CURRENT_LIMIT_AMPS);
        telemetry.addData("Slide status", faulted ? "FAULT: " + faultReason : "OK");
    }

    public double getLeftPower() {
        return leftSlide.getPower();
    }

    public double getRightPower() {
        return rightSlide.getPower();
    }

    private void moveToPosition(int targetTicks) {
        attemptRecovery();
        if (faulted) {
            stop();
            return;
        }
        targetPositionTicks = enforceLimits(targetTicks);

        leftSlide.setTargetPosition(targetPositionTicks);
        rightSlide.setTargetPosition(targetPositionTicks);

        leftSlide.setMode(RunMode.RUN_TO_POSITION);
        rightSlide.setMode(RunMode.RUN_TO_POSITION);

        // FTC SDK runs a built-in P controller in RUN_TO_POSITION; tune power + target values above
        applyPower(RobotConstants.SLIDE_POWER);
    }

    private int enforceLimits(int desiredTicks) {
        if (desiredTicks > RobotConstants.SLIDE_MAX) {
            return RobotConstants.SLIDE_MAX;
        }
        if (desiredTicks < RobotConstants.SLIDE_INTAKE) {
            return RobotConstants.SLIDE_INTAKE;
        }
        return desiredTicks;
    }

    private void applyPower(double requestedPower) {
        double safePower = applySlowZone(requestedPower);
        safePower = enforceSafety(safePower);
        lastCommandedPower = safePower;
        leftSlide.setPower(safePower);
        rightSlide.setPower(safePower);
    }

    private double applySlowZone(double requestedPower) {
        double avgPosition = getAveragePosition();
        boolean nearingTop = avgPosition > RobotConstants.SLIDE_MAX - RobotConstants.SLIDE_SLOW_ZONE_TICKS && requestedPower > 0;
        boolean nearingBottom = avgPosition < RobotConstants.SLIDE_INTAKE + RobotConstants.SLIDE_SLOW_ZONE_TICKS && requestedPower < 0;
        if (nearingTop || nearingBottom) {
            return requestedPower * RobotConstants.SLIDE_SLOW_ZONE_SCALE;
        }
        return requestedPower;
    }

    private double enforceSafety(double requestedPower) {
        double boundedPower = Range.clip(requestedPower, -1.0, 1.0);
        long now = System.currentTimeMillis();

        if (faulted) {
            // keep power cut until the fault clears and currents are calm
            if ((now - faultTimestampMs) > RobotConstants.SLIDE_FAULT_CLEAR_MS
                    && getAverageCurrent() < RobotConstants.SLIDE_CURRENT_LIMIT_AMPS * RobotConstants.SLIDE_RECOVERY_CURRENT_RATIO) {
                faulted = false;
                faultReason = "";
            } else {
                return 0.0;
            }
        }

        double avgCurrent = Math.abs(getAverageCurrent());
        if (avgCurrent > RobotConstants.SLIDE_CURRENT_LIMIT_AMPS) {
            triggerFault("Over current");
            return 0.0;
        }

        double avgVelocity = Math.abs(getAverageVelocity());
        double commandedPower = Math.abs(boundedPower);
        if (commandedPower > RobotConstants.SLIDE_STALL_MIN_POWER) {
            if (avgVelocity < RobotConstants.SLIDE_STALL_VELOCITY_TICKS_PER_S) {
                if (!stallTimerRunning) {
                    stallTimer.reset();
                    stallTimerRunning = true;
                }
                if (stallTimer.milliseconds() > RobotConstants.SLIDE_STALL_TIMEOUT_MS) {
                    triggerFault("Slide stall detected");
                    return 0.0;
                }
            } else {
                stallTimer.reset();
                stallTimerRunning = false;
            }
        } else {
            stallTimer.reset();
            stallTimerRunning = false;
        }

        return boundedPower;
    }

    private void triggerFault(String reason) {
        faulted = true;
        faultReason = reason;
        faultTimestampMs = System.currentTimeMillis();
        stallTimer.reset();
        stallTimerRunning = false;
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    private void attemptRecovery() {
        enforceSafety(lastCommandedPower);
    }
}
