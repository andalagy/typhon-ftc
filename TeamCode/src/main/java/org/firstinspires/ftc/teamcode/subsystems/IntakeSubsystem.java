package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Handles the dual intake rollers so we can independently move storage and feed.
 * Power is kept simple on purpose: full power in, full power out, or stopped.
 * The motor names come straight from RobotConstants so they match the config file.
 */
public class IntakeSubsystem {
    private final DcMotor storageMotor;
    private final DcMotor feedMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        storageMotor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_STORAGE_NAME);
        feedMotor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_FEED_NAME);

        storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        storageMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feedMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        stop();
    }

    public void intakeIn() {
        storageMotor.setPower(1.0);
        feedMotor.setPower(1.0);
    }

    public void intakeOut() {
        storageMotor.setPower(-1.0);
        feedMotor.setPower(-1.0);
    }

    public void storageIn() {
        storageMotor.setPower(1.0);
    }

    public void storageOut() {
        storageMotor.setPower(-1.0);
    }

    public void stopStorage() {
        storageMotor.setPower(0);
    }

    public void feedIn() {
        feedMotor.setPower(1.0);
    }

    public void feedOut() {
        feedMotor.setPower(-1.0);
    }

    public void stopFeed() {
        feedMotor.setPower(0);
    }

    public void stop() {
        storageMotor.setPower(0);
        feedMotor.setPower(0);
    }

    public double getStoragePower() {
        return storageMotor.getPower();
    }

    public double getFeedPower() {
        return feedMotor.getPower();
    }
}
