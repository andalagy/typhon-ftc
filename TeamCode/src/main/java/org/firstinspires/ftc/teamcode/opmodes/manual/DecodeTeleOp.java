package org.firstinspires.ftc.teamcode.opmodes.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.DetectedMotif;

/**
 * Driver-controlled program that wires gamepad inputs into the drive, intake, and vision.
 * Uses the subsystem classes so hardware setup stays in one place and the OpMode stays readable.
 * Field-centric drive math keeps controls intuitive while comments explain the flow for new teammates.
 */
@TeleOp(name = "Decode TeleOp", group = "Main")
public class DecodeTeleOp extends LinearOpMode {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private VisionSubsystem vision;

    private boolean headingHoldToggleLatch = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Build each subsystem so we reuse the same hardware mapping everywhere
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        vision.start();
        vision.useAprilTags();
        telemetry.addLine("TeleOp ready — press play when the field says go");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        // zero heading before moving so field-centric drive lines up with the real field
        drive.resetHeading();
        drive.enableHeadingHold(false);

        while (opModeIsActive()) {
            // Drive control: left stick moves the robot around the field, right stick rotates it like a car joystick
            double y = -gamepad1.left_stick_y; // forward on the stick is negative in FTC, so flip it
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;
            boolean slowMode = gamepad1.right_stick_button; // hold for gentle mode when lining up
            if (gamepad1.y) {
                drive.resetHeading(); // quick re-zero if field-centric starts to feel off
            }
            boolean headingHoldButton = gamepad1.x;
            if (headingHoldButton && !headingHoldToggleLatch) {
                drive.enableHeadingHold(!drive.isHeadingHoldEnabled());
            }
            headingHoldToggleLatch = headingHoldButton;

            drive.drive(x, y, rotation, slowMode);

            // Storage intake: left bumper pulls in, left trigger reverses
            if (gamepad1.left_bumper) {
                intake.storageIn();
            } else if (gamepad1.left_trigger > 0.2) {
                intake.storageOut();
            } else {
                intake.stopStorage();
            }

            // Feed intake: right bumper pulls in, right trigger reverses
            if (gamepad1.right_bumper) {
                intake.feedIn();
            } else if (gamepad1.right_trigger > 0.2) {
                intake.feedOut();
            } else {
                intake.stopFeed();
            }

            // Hood control: D-pad up/down to adjust angle, no input stops the motor
            if (gamepad2.dpad_up) {
                hood.hoodUp();
            } else if (gamepad2.dpad_down) {
                hood.hoodDown();
            } else {
                hood.stop();
            }

            // Vision heartbeat: report live motif
            DetectedMotif detectedMotif = vision.getCurrentMotif();
            VisionSubsystem.BackdropTarget target = vision.getBackdropTarget(detectedMotif);
            vision.applyCameraControls();

            telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("Heading hold", drive.isHeadingHoldEnabled());
            telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
            telemetry.addData("Drive powers", "FL %.2f FR %.2f BL %.2f BR %.2f",
                    drive.getFrontLeftPower(), drive.getFrontRightPower(),
                    drive.getBackLeftPower(), drive.getBackRightPower());
            telemetry.addData("Storage intake", "%.2f", intake.getStoragePower());
            telemetry.addData("Feed intake", "%.2f", intake.getFeedPower());
            telemetry.addData("Hood power", "%.2f", hood.getPower());
            telemetry.addData("Hood ticks", hood.getPositionTicks());
            telemetry.addData("Vision motif", detectedMotif);
            telemetry.addData("Vision camera", vision.getCameraStatus());
            telemetry.addData("Tag visible", target != null);
            telemetry.update();
        }

        // make sure everything is stopped once the driver hits stop
        drive.stop();
        intake.stop();
        hood.stop();
        vision.stop();
    }

    private double getBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage == Double.POSITIVE_INFINITY ? 0.0 : minVoltage;
    }
}
