package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem.SlidePreset;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.DetectedMotif;

/**
 * Driver-controlled program that wires gamepad inputs into the drive, intake, slides, and gate.
 * Uses the subsystem classes so hardware setup stays in one place and the OpMode stays readable.
 * Field-centric drive math keeps controls intuitive while comments explain the flow for new teammates.
 */
@TeleOp(name = "Decode TeleOp", group = "Main")
public class DecodeTeleOp extends LinearOpMode {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private SlideSubsystem slides;
    private GateSubsystem gate;
    private VisionSubsystem vision;

    private enum MacroType {NONE, RAPID_CYCLE, HANG, PARK}
    private enum MacroStage {IDLE, COLLECTING, LIFTING, SCORING, RETURNING, HANG_EXTENDING, HANG_HOLDING, PARKING}
    private MacroType activeMacro = MacroType.NONE;
    private MacroStage macroStage = MacroStage.IDLE;
    private final ElapsedTime macroTimer = new ElapsedTime();

    private boolean headingHoldToggleLatch = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Build each subsystem so we reuse the same hardware mapping everywhere
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        slides = new SlideSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        gate.close();
        vision.start();
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

            // Slide control + macros
            double slideInput = -gamepad2.left_stick_y; // push up to extend, pull down to retract
            boolean rapidCycleRequested = gamepad2.y;
            boolean hangRequested = gamepad2.dpad_up;
            boolean parkRequested = gamepad2.dpad_down;
            boolean cancelRequested = gamepad2.left_bumper || Math.abs(slideInput) > 0.1 || gamepad2.a || gamepad2.b;

            if (slides.isFaulted() && activeMacro != MacroType.NONE) {
                cancelMacro();
            }

            if (cancelRequested && activeMacro != MacroType.NONE) {
                cancelMacro();
            }

            if (activeMacro == MacroType.NONE) {
                // Intake control: right trigger sucks game pieces in, left trigger spits them back out
                if (gamepad2.right_trigger > 0.1) {
                    intake.intakeIn();
                } else if (gamepad2.left_trigger > 0.1) {
                    intake.intakeOut();
                } else {
                    intake.stop();
                }

                // Gate control: A opens the bucket gate, B closes it
                if (gamepad2.a) {
                    gate.open();
                } else if (gamepad2.b) {
                    gate.close();
                }

                if (rapidCycleRequested) {
                    startRapidCycle();
                } else if (hangRequested) {
                    startHangMacro();
                } else if (parkRequested) {
                    startParkMacro();
                } else {
                    slides.manualControl(slideInput);
                }
            } else {
                runMacro();
            }

            // Vision heartbeat: report live motif
            DetectedMotif detectedMotif = vision.getCurrentMotif();
            vision.applyCameraControls();

            telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("Heading hold", drive.isHeadingHoldEnabled());
            telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
            telemetry.addData("Macro", activeMacro + " / " + macroStage);
            telemetry.addData("Slide at target?", slides.isAtTarget());
            telemetry.addData("Drive powers", "FL %.2f FR %.2f BL %.2f BR %.2f",
                    drive.getFrontLeftPower(), drive.getFrontRightPower(),
                    drive.getBackLeftPower(), drive.getBackRightPower());
            slides.addTelemetry(telemetry);
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            telemetry.addData("Gate position", "%.2f", gate.getPosition());
            telemetry.addData("Vision motif", detectedMotif);
            telemetry.addData("Vision camera", vision.getCameraStatus());
            telemetry.update();
        }

        // make sure everything is stopped once the driver hits stop
        drive.stop();
        intake.stop();
        slides.stop();
        gate.close();
        vision.stop();
    }

    private void startRapidCycle() {
        activeMacro = MacroType.RAPID_CYCLE;
        macroStage = MacroStage.COLLECTING;
        macroTimer.reset();
        gate.close();
        intake.intakeIn();
        slides.goToPreset(SlidePreset.INTAKE);
    }

    private void startHangMacro() {
        activeMacro = MacroType.HANG;
        macroStage = MacroStage.HANG_EXTENDING;
        macroTimer.reset();
        intake.stop();
        gate.close();
        slides.goToPreset(SlidePreset.MAX);
    }

    private void startParkMacro() {
        activeMacro = MacroType.PARK;
        macroStage = MacroStage.PARKING;
        macroTimer.reset();
        intake.stop();
        gate.close();
        slides.goToPreset(SlidePreset.INTAKE);
    }

    private void cancelMacro() {
        activeMacro = MacroType.NONE;
        macroStage = MacroStage.IDLE;
        intake.stop();
        gate.close();
    }

    private void runMacro() {
        if (slides.isFaulted()) {
            activeMacro = MacroType.NONE;
            macroStage = MacroStage.IDLE;
            return;
        }
        switch (activeMacro) {
            case RAPID_CYCLE:
                runRapidCycle();
                break;
            case HANG:
                runHangMacro();
                break;
            case PARK:
                runParkMacro();
                break;
            case NONE:
            default:
                break;
        }
    }

    private void runRapidCycle() {
        switch (macroStage) {
            case COLLECTING:
                // keep pulling in until we give the slides a moment to settle at intake
                if (macroTimer.milliseconds() > 400) {
                    intake.stop();
                    slides.goToPreset(SlidePreset.HIGH);
                    macroStage = MacroStage.LIFTING;
                }
                break;
            case LIFTING:
                if (slides.isAtTarget()) {
                    gate.open();
                    macroTimer.reset();
                    macroStage = MacroStage.SCORING;
                }
                break;
            case SCORING:
                if (macroTimer.milliseconds() > 600) {
                    gate.close();
                    slides.goToPreset(SlidePreset.INTAKE);
                    macroTimer.reset();
                    macroStage = MacroStage.RETURNING;
                }
                break;
            case RETURNING:
                if (slides.isAtTarget()) {
                    intake.stop();
                    activeMacro = MacroType.NONE;
                    macroStage = MacroStage.IDLE;
                }
                break;
            case IDLE:
            default:
                activeMacro = MacroType.NONE;
                macroStage = MacroStage.IDLE;
                break;
        }
    }

    private void runHangMacro() {
        switch (macroStage) {
            case HANG_EXTENDING:
                if (slides.isAtTarget()) {
                    macroStage = MacroStage.HANG_HOLDING;
                }
                break;
            case HANG_HOLDING:
                // Stay extended while drivers maneuver; allow cancel to bail out
                break;
            default:
                activeMacro = MacroType.NONE;
                macroStage = MacroStage.IDLE;
                break;
        }
    }

    private void runParkMacro() {
        switch (macroStage) {
            case PARKING:
                if (slides.isAtTarget()) {
                    activeMacro = MacroType.NONE;
                    macroStage = MacroStage.IDLE;
                }
                break;
            default:
                activeMacro = MacroType.NONE;
                macroStage = MacroStage.IDLE;
                break;
        }
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
