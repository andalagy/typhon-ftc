package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.external.Telemetry;
import com.qualcomm.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Comparator;
import java.util.List;

/**
 * Limelight-backed vision helper that reports sleeve/signal motifs and AprilTag pose offsets.
 *
 * Pipeline indices are configured in {@link RobotConstants}:
 * - {@link RobotConstants#LIMELIGHT_PIPELINE_MOTIF}
 * - {@link RobotConstants#LIMELIGHT_PIPELINE_APRILTAG}
 *
 * For motif detection, the preferred flow is a Python SnapScript that writes the motif index to
 * pythonOutput[0]: 0 -> MOTIF_A, 1 -> MOTIF_B, 2 -> MOTIF_C.
 */
@Config
public class VisionSubsystem {

    public enum DetectedMotif {
        MOTIF_A,
        MOTIF_B,
        MOTIF_C
    }

    public enum AprilTagMeaning {
        BLUE_GOAL(20, "Blue Goal"),
        PATTERN_GPP(21, "G P P"),
        PATTERN_PGP(22, "P G P"),
        PATTERN_PPG(23, "P P G"),
        RED_GOAL(24, "Red Goal"),
        UNKNOWN(-1, "Unknown");

        private final int tagId;
        private final String description;

        AprilTagMeaning(int tagId, String description) {
            this.tagId = tagId;
            this.description = description;
        }

        public int getTagId() {
            return tagId;
        }

        public String getDescription() {
            return description;
        }

        public static AprilTagMeaning fromId(int tagId) {
            switch (tagId) {
                case 20:
                    return BLUE_GOAL;
                case 21:
                    return PATTERN_GPP;
                case 22:
                    return PATTERN_PGP;
                case 23:
                    return PATTERN_PPG;
                case 24:
                    return RED_GOAL;
                default:
                    return UNKNOWN;
            }
        }
    }

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private volatile String cameraStatus = "Not started";
    private volatile DetectedMotif currentMotif = DetectedMotif.MOTIF_A;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        Limelight3A device;
        try {
            device = hardwareMap.get(Limelight3A.class, RobotConstants.LIMELIGHT_NAME);
        } catch (RuntimeException e) {
            device = null;
            cameraStatus = "Limelight not found: " + e.getMessage();
            pushTelemetry(cameraStatus);
        }
        limelight = device;
    }

    /** start streaming to the RC phone and begin detecting motifs */
    public void start() {
        if (limelight == null) {
            cameraStatus = "Limelight not available";
            pushTelemetry(cameraStatus);
            return;
        }
        try {
            limelight.setPollRateHz(100);
            limelight.start();
            cameraStatus = "Running";
            pushTelemetry("Limelight running");
        } catch (RuntimeException e) {
            cameraStatus = "Start error: " + e.getMessage();
            pushTelemetry(cameraStatus);
        }
    }

    /** stop streaming to free the camera for other OpModes */
    public void stop() {
        if (limelight == null) {
            return;
        }
        try {
            limelight.stop();
        } catch (RuntimeException e) {
            cameraStatus = "Stop error: " + e.getMessage();
            pushTelemetry(cameraStatus);
        }
    }

    /** latest classification from the pipeline; safe to call from any OpMode loop */
    public DetectedMotif getCurrentMotif() {
        updateMotifFromResult(getLatestResult());
        return currentMotif;
    }

    /** latest AprilTag target pose (if any) */
    public BackdropTarget getBackdropTarget(DetectedMotif desired) {
        return chooseBackdropTarget(getLatestResult(), desired);
    }

    /** latest AprilTag meaning (pattern/goal) based on detected ID */
    public AprilTagMeaning getLatestAprilTagMeaning() {
        return chooseTagMeaning(getLatestResult());
    }

    /** switch from the sleeve pipeline to the AprilTag pipeline for backdrop alignment */
    public void useAprilTags() {
        if (limelight != null) {
            limelight.pipelineSwitch(RobotConstants.LIMELIGHT_PIPELINE_APRILTAG);
        }
    }

    /** revert to sleeve detection (e.g., for debug) */
    public void useSleevePipeline() {
        if (limelight != null) {
            limelight.pipelineSwitch(RobotConstants.LIMELIGHT_PIPELINE_MOTIF);
        }
    }

    /** latest camera state for telemetry */
    public String getCameraStatus() {
        return cameraStatus;
    }

    /**
     * Limelight exposure/white balance are managed in the Limelight web UI, so this is a no-op.
     */
    public void applyCameraControls() {
        if (limelight == null) {
            cameraStatus = "Limelight not available";
        } else {
            cameraStatus = "Controls managed in Limelight UI";
        }
    }

    private void updateMotifFromResult(LLResult result) {
        if (result == null) {
            currentMotif = DetectedMotif.MOTIF_A;
            return;
        }

        double[] pythonOutput = result.getPythonOutput();
        if (pythonOutput != null && pythonOutput.length > 0) {
            currentMotif = motifFromIndex(pythonOutput[0]);
            return;
        }

        List<ColorResult> colors = result.getColorResults();
        if (colors == null || colors.isEmpty()) {
            currentMotif = DetectedMotif.MOTIF_A;
            return;
        }

        ColorResult best = colors.stream()
                .max(Comparator.comparingDouble(VisionSubsystem::colorArea))
                .orElse(null);
        if (best == null) {
            currentMotif = DetectedMotif.MOTIF_A;
            return;
        }
        int index = colors.indexOf(best);
        currentMotif = motifFromIndex(index);
    }

    private static double colorArea(ColorResult result) {
        if (result == null) {
            return 0.0;
        }
        return result.getArea();
    }

    private static DetectedMotif motifFromIndex(double index) {
        int rounded = (int) Math.round(index);
        switch (rounded) {
            case 1:
                return DetectedMotif.MOTIF_B;
            case 2:
                return DetectedMotif.MOTIF_C;
            case 0:
            default:
                return DetectedMotif.MOTIF_A;
        }
    }

    private BackdropTarget chooseBackdropTarget(LLResult result, DetectedMotif desired) {
        if (result == null) {
            return null;
        }
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        int expectedId = desired == DetectedMotif.MOTIF_A ? 1
                : desired == DetectedMotif.MOTIF_B ? 2 : 3;

        FiducialResult bestExpected = null;
        FiducialResult bestAny = null;
        double bestExpectedRange = Double.POSITIVE_INFINITY;
        double bestAnyRange = Double.POSITIVE_INFINITY;

        for (FiducialResult fiducial : fiducials) {
            if (fiducial == null) {
                continue;
            }
            int id = fiducial.getFiducialId();
            if (id < 1 || id > 3) {
                continue;
            }
            double range = estimateRangeMeters(fiducial);
            if (range < bestAnyRange) {
                bestAnyRange = range;
                bestAny = fiducial;
            }
            if (id == expectedId && range < bestExpectedRange) {
                bestExpectedRange = range;
                bestExpected = fiducial;
            }
        }

        FiducialResult chosen = bestExpected != null ? bestExpected : bestAny;
        if (chosen == null) {
            return null;
        }

        Pose3D pose = chosen.getRobotPoseTargetSpace();
        double lateralMeters = 0.0;
        double rangeMeters = estimateRangeMeters(chosen);
        double headingErrorRad = 0.0;

        if (pose != null) {
            VectorF translation = pose.getPosition();
            if (translation != null && translation.length() >= 3) {
                lateralMeters = translation.get(1);
                rangeMeters = translation.get(2);
            }
            Orientation orientation = pose.getOrientation();
            if (orientation != null) {
                headingErrorRad = orientation.angleUnit == AngleUnit.RADIANS
                        ? orientation.thirdAngle
                        : Math.toRadians(orientation.thirdAngle);
            }
        }

        double xPixelError = result.getTx();
        return new BackdropTarget(chosen.getFiducialId(), rangeMeters, lateralMeters,
                headingErrorRad, xPixelError);
    }

    private static double estimateRangeMeters(FiducialResult fiducial) {
        if (fiducial == null) {
            return Double.POSITIVE_INFINITY;
        }
        Pose3D pose = fiducial.getRobotPoseTargetSpace();
        if (pose == null) {
            return Double.POSITIVE_INFINITY;
        }
        VectorF translation = pose.getPosition();
        if (translation == null || translation.length() < 3) {
            return Double.POSITIVE_INFINITY;
        }
        double y = translation.get(1);
        double z = translation.get(2);
        return Math.hypot(y, z);
    }

    private AprilTagMeaning chooseTagMeaning(LLResult result) {
        if (result == null) {
            return AprilTagMeaning.UNKNOWN;
        }
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return AprilTagMeaning.UNKNOWN;
        }
        FiducialResult closest = fiducials.stream()
                .filter(fiducial -> fiducial != null)
                .min(Comparator.comparingDouble(VisionSubsystem::estimateRangeMeters))
                .orElse(null);
        if (closest == null) {
            return AprilTagMeaning.UNKNOWN;
        }
        return AprilTagMeaning.fromId(closest.getFiducialId());
    }

    private LLResult getLatestResult() {
        if (limelight == null) {
            cameraStatus = "Limelight not available";
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            cameraStatus = "No result";
            return null;
        }

        String status = "Running";
        List<FiducialResult> fiducials = result.getFiducialResults();
        List<ColorResult> colors = result.getColorResults();
        boolean hasTargets = (fiducials != null && !fiducials.isEmpty())
                || (colors != null && !colors.isEmpty());
        if (!hasTargets) {
            status = "No targets";
        }

        long staleness = result.getStaleness();
        if (staleness >= 0) {
            status = String.format("%s (stale %dms)", status, staleness);
        }
        cameraStatus = status;
        return result;
    }

    private void pushTelemetry(String message) {
        if (telemetry != null) {
            telemetry.addLine(message);
            telemetry.update();
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            dashboard.getTelemetry().addLine(message);
            dashboard.getTelemetry().update();
        }
    }

    /** Pose/offset helper for aligning to the backdrop tag columns */
    public static class BackdropTarget {
        public final int tagId;
        public final double rangeMeters;
        public final double lateralMeters;
        public final double headingErrorRad;
        public final double xPixelError;

        public BackdropTarget(int tagId, double rangeMeters, double lateralMeters,
                              double headingErrorRad, double xPixelError) {
            this.tagId = tagId;
            this.rangeMeters = rangeMeters;
            this.lateralMeters = lateralMeters;
            this.headingErrorRad = headingErrorRad;
            this.xPixelError = xPixelError;
        }

        public double getLateralInches() {
            return lateralMeters * 39.3701;
        }

        public double getRangeInches() {
            return rangeMeters * 39.3701;
        }
    }
}
