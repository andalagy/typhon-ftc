package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Handles the intake roller so we can grab or spit out game pieces on command.
 * Power is kept simple on purpose: full power in, full power out, or stopped.
 * The motor name comes straight from RobotConstants so it matches the config file.
 */
public class IntakeSubsystem {
    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intakeIn() {
        intakeMotor.setPower(1.0);
    }

    public void intakeOut() {
        intakeMotor.setPower(-1.0);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }
}
