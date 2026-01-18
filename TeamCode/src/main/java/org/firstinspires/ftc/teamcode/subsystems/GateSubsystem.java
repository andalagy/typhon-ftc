package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Runs the bucket gate servo that keeps samples in place until we want to score.
 * Call open() to drop game pieces and close() to lock everything back down.
 * Positions live in RobotConstants so tuning is easy without digging through code.
 */
public class GateSubsystem {
    private final Servo gateServo;

    public GateSubsystem(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, RobotConstants.GATE_SERVO_NAME);
    }

    public void open() {
        gateServo.setPosition(RobotConstants.GATE_OPEN);
    }

    public void close() {
        gateServo.setPosition(RobotConstants.GATE_CLOSED);
    }

    /** quick dump flick; call in a loop so the servo actually gets time to move. */
    public void dump() {
        open();
    }

    public double getPosition() {
        return gateServo.getPosition();
    }
}
