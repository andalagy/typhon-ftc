package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Controls the hood angle for shooting. Uses encoder limits when available,
 * and falls back to a timed safety cutoff if limits are not configured.
 */
public class HoodSubsystem {
    private final DcMotor hoodMotor;
    private final ElapsedTime runTimer = new ElapsedTime();
    private int lastDirection = 0;

    private final int minTicks = RobotConstants.HOOD_MIN_TICKS;
    private final int maxTicks = RobotConstants.HOOD_MAX_TICKS;
    private final boolean useEncoderLimits = maxTicks > minTicks;

    public HoodSubsystem(HardwareMap hardwareMap) {
        hoodMotor = hardwareMap.get(DcMotor.class, RobotConstants.HOOD_MOTOR_NAME);
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hoodMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void hoodUp() {
        setPower(Math.abs(RobotConstants.HOOD_POWER));
    }

    public void hoodDown() {
        setPower(-Math.abs(RobotConstants.HOOD_POWER));
    }

    public void stop() {
        hoodMotor.setPower(0.0);
        lastDirection = 0;
        runTimer.reset();
    }

    public int getPositionTicks() {
        return hoodMotor.getCurrentPosition();
    }

    public double getPower() {
        return hoodMotor.getPower();
    }

    private void setPower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        int direction = (int) Math.signum(power);

        if (direction == 0) {
            stop();
            return;
        }

        if (useEncoderLimits) {
            int position = hoodMotor.getCurrentPosition();
            if (direction > 0 && position >= maxTicks) {
                hoodMotor.setPower(0.0);
                return;
            }
            if (direction < 0 && position <= minTicks) {
                hoodMotor.setPower(0.0);
                return;
            }
        } else {
            if (direction != lastDirection) {
                runTimer.reset();
                lastDirection = direction;
            }
            if (runTimer.seconds() >= RobotConstants.HOOD_MAX_RUN_SEC) {
                hoodMotor.setPower(0.0);
                return;
            }
        }

        hoodMotor.setPower(power);
    }
}
