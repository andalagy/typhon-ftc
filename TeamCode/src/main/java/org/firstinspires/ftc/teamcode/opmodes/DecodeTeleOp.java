package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretAimingSubsystem;
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
    private TurretAimingSubsystem turret;
    private VisionSubsystem vision;

    private boolean headingHoldToggleLatch = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Build each subsystem so we reuse the same hardware mapping everywhere
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        turret = new TurretAimingSubsystem(hardwareMap, telemetry);

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
            boolean slowMode = gamepad1.right_bumper; // hold for gentle mode when lining up
            if (gamepad1.left_bumper) {
                drive.resetHeading(); // quick re-zero if field-centric starts to feel off
            }
            boolean headingHoldButton = gamepad1.x;
            if (headingHoldButton && !headingHoldToggleLatch) {
                drive.enableHeadingHold(!drive.isHeadingHoldEnabled());
            }
            headingHoldToggleLatch = headingHoldButton;

            drive.drive(x, y, rotation, slowMode);

            // Intake control: right trigger sucks game pieces in, left trigger spits them back out
            if (gamepad2.right_trigger > 0.1) {
                intake.intakeIn();
            } else if (gamepad2.left_trigger > 0.1) {
                intake.intakeOut();
            } else {
                intake.stop();
            }

            // Vision heartbeat: report live motif
            DetectedMotif detectedMotif = vision.getCurrentMotif();
            VisionSubsystem.BackdropTarget target = vision.getBackdropTarget(detectedMotif);
            if (target != null) {
                turret.setVisionAim(target.headingErrorRad, target.xPixelError, true);
            } else {
                turret.setVisionAim(0.0, 0.0, false);
            }
            turret.update();
            vision.applyCameraControls();

            telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("Heading hold", drive.isHeadingHoldEnabled());
            telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
            telemetry.addData("Drive powers", "FL %.2f FR %.2f BL %.2f BR %.2f",
                    drive.getFrontLeftPower(), drive.getFrontRightPower(),
                    drive.getBackLeftPower(), drive.getBackRightPower());
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            telemetry.addData("Vision motif", detectedMotif);
            telemetry.addData("Vision camera", vision.getCameraStatus());
            telemetry.addData("Tag visible", target != null);
            telemetry.addData("Turret aligned", turret.isAligned());
            telemetry.addData("Turret aim error", "%.3f", turret.getAimError());
            telemetry.update();
        }

        // make sure everything is stopped once the driver hits stop
        drive.stop();
        intake.stop();
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
