package org.firstinspires.ftc.teamcode.subsystems;

import android.content.res.Resources;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.external.Telemetry;
import com.qualcomm.robotcore.external.hardware.camera.controls.ExposureControl;
import com.qualcomm.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.WebcamName;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * EasyOpenCV-backed vision helper that streams from a webcam and classifies the sleeve/signal motif.
 * The pipeline uses simple color thresholds in three regions of interest and reports MOTIF_A/B/C.
 * Replace the thresholds/ROIs as you tune on the real field — the public API stays the same.
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

    private final OpenCvWebcam webcam;
    private final SleevePipeline pipeline;
    private final AprilTagPipeline aprilTagPipeline;
    private final Telemetry telemetry;
    private volatile String cameraStatus = "Not started";

    // Dashboard/telemetry-tunable HSV bounds and ROI placement
    public static double[] LOWER_BLUE = {90, 60, 50};
    public static double[] UPPER_BLUE = {140, 255, 255};
    public static double[] LOWER_GREEN = {40, 50, 50};
    public static double[] UPPER_GREEN = {85, 255, 255};
    public static double[] LOWER_RED1 = {0, 70, 50};
    public static double[] UPPER_RED1 = {10, 255, 255};
    public static double[] LOWER_RED2 = {170, 70, 50};
    public static double[] UPPER_RED2 = {180, 255, 255};

    public static int ROI_Y = 200;
    public static int ROI_WIDTH = 160;
    public static int ROI_HEIGHT = 120;
    public static int LEFT_X = 40;
    public static int CENTER_X = 240;
    public static int RIGHT_X = 440;

    // AprilTag/backdrop search parameters (dashboard tunable)
    public static double TAG_SIZE_METERS = 0.0508; // 2 inches
    public static double FX = 578.272;
    public static double FY = 578.272;
    public static double CX = 402.145;
    public static double CY = 221.506;
    public static double TAG_DECIMATION = 2.0;
    public static int TAG_ROI_X = 40;
    public static int TAG_ROI_Y = 120;
    public static int TAG_ROI_WIDTH = 560;
    public static int TAG_ROI_HEIGHT = 240;

    // Camera control toggles
    public static boolean USE_MANUAL_EXPOSURE = true;
    public static double EXPOSURE_MS = 12.0;
    public static boolean USE_MANUAL_WHITE_BALANCE = true;
    public static int WHITE_BALANCE_KELVIN = 4500;

    // Filtering knobs
    public static double SCORE_SMOOTHING = 0.6; // closer to 1.0 = more smoothing
    public static double CONTOUR_WEIGHT = 0.35; // fraction of score that comes from contour area

    public VisionSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        Resources res = hardwareMap.appContext.getResources();
        int cameraMonitorViewId = res.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, RobotConstants.WEBCAM_NAME),
                cameraMonitorViewId);
        pipeline = new SleevePipeline();
        webcam.setPipeline(pipeline);
        aprilTagPipeline = new AprilTagPipeline();
    }

    /** start streaming to the RC phone and begin detecting motifs */
    public void start() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                applyCameraControls();
                cameraStatus = "Streaming";
                pushTelemetry("Camera opened and streaming");
            }

            @Override
            public void onError(int errorCode) {
                cameraStatus = "Open error: " + errorCode;
                pushTelemetry(cameraStatus);
            }
        });
    }

    /** stop streaming to free the camera for other OpModes */
    public void stop() {
        webcam.stopStreaming();
        webcam.closeCameraDeviceAsync(() -> { });
    }

    /** latest classification from the pipeline; safe to call from any OpMode loop */
    public DetectedMotif getCurrentMotif() {
        return pipeline.getCurrentMotif();
    }

    /** latest AprilTag target pose (if any) */
    public BackdropTarget getBackdropTarget(DetectedMotif desired) {
        return aprilTagPipeline.getBestTargetFor(desired);
    }

    /** latest AprilTag meaning (pattern/goal) based on detected ID */
    public AprilTagMeaning getLatestAprilTagMeaning() {
        return aprilTagPipeline.getLatestTagMeaning();
    }

    /** switch from the sleeve pipeline to the AprilTag pipeline for backdrop alignment */
    public void useAprilTags() {
        if (webcam != null) {
            webcam.setPipeline(aprilTagPipeline);
            aprilTagPipeline.updateDecimation();
        }
    }

    /** revert to sleeve detection (e.g., for debug) */
    public void useSleevePipeline() {
        if (webcam != null) {
            webcam.setPipeline(pipeline);
        }
    }

    /** latest camera state for telemetry */
    public String getCameraStatus() {
        return cameraStatus;
    }

    /**
     * Apply manual exposure/white balance controls. Safe to call repeatedly (e.g. after slider tweaks).
     */
    public void applyCameraControls() {
        if (webcam == null || !webcam.isStreaming()) {
            return;
        }

        if (USE_MANUAL_EXPOSURE) {
            try {
                ExposureControl exposureControl = webcam.getExposureControl();
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
            } catch (RuntimeException e) {
                pushTelemetry("Exposure control failed: " + e.getMessage());
            }
        }

        if (USE_MANUAL_WHITE_BALANCE) {
            try {
                WhiteBalanceControl wbControl = webcam.getWhiteBalanceControl();
                wbControl.setMode(WhiteBalanceControl.Mode.MANUAL);
                wbControl.setWhiteBalanceTemperature(WHITE_BALANCE_KELVIN);
            } catch (RuntimeException e) {
                pushTelemetry("White balance control failed: " + e.getMessage());
            }
        }
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

    /**
     * Example pipeline that checks three side-by-side ROIs for dominant color.
     * Tune the HSV bounds for your sleeve/signal art and adjust the rectangles to match the frame.
     */
    private static class SleevePipeline extends OpenCvPipeline {
        private DetectedMotif currentMotif = DetectedMotif.MOTIF_A;
        private double leftAvg = 0;
        private double centerAvg = 0;
        private double rightAvg = 0;

        private Rect clampRect(Rect roi, Mat input) {
            int x = Math.max(0, roi.x);
            int y = Math.max(0, roi.y);
            int width = Math.min(roi.width, input.width() - x);
            int height = Math.min(roi.height, input.height() - y);
            return new Rect(x, y, Math.max(0, width), Math.max(0, height));
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Rect leftRoi = clampRect(new Rect(LEFT_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT), hsv);
            Rect centerRoi = clampRect(new Rect(CENTER_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT), hsv);
            Rect rightRoi = clampRect(new Rect(RIGHT_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT), hsv);

            double leftScore = computeScore(hsv.submat(leftRoi));
            double centerScore = computeScore(hsv.submat(centerRoi));
            double rightScore = computeScore(hsv.submat(rightRoi));

            leftAvg = smooth(leftAvg, leftScore);
            centerAvg = smooth(centerAvg, centerScore);
            rightAvg = smooth(rightAvg, rightScore);

            // choose the brightest ROI as the detected motif
            if (leftAvg > centerAvg && leftAvg > rightAvg) {
                currentMotif = DetectedMotif.MOTIF_A;
            } else if (centerAvg > rightAvg) {
                currentMotif = DetectedMotif.MOTIF_B;
            } else {
                currentMotif = DetectedMotif.MOTIF_C;
            }

            // draw debug overlays so tuning is easier on the RC preview
            Imgproc.rectangle(input, leftRoi, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRoi, new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(input, rightRoi, new Scalar(0, 0, 255), 2);
            Imgproc.putText(input, currentMotif.name(), new Point(20, 40),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 255), 2);
            Imgproc.putText(input, String.format("L%.0f C%.0f R%.0f", leftAvg, centerAvg, rightAvg),
                    new Point(20, 70), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 0), 2);

            hsv.release();
            return input;
        }

        private double computeScore(Mat roi) {
            // blend multiple colors so different sleeve palettes still classify
            Scalar lowerBlue = new Scalar(LOWER_BLUE);
            Scalar upperBlue = new Scalar(UPPER_BLUE);
            Scalar lowerGreen = new Scalar(LOWER_GREEN);
            Scalar upperGreen = new Scalar(UPPER_GREEN);
            Scalar lowerRed1 = new Scalar(LOWER_RED1);
            Scalar upperRed1 = new Scalar(UPPER_RED1);
            Scalar lowerRed2 = new Scalar(LOWER_RED2);
            Scalar upperRed2 = new Scalar(UPPER_RED2);

            Mat blueMask = new Mat();
            Mat greenMask = new Mat();
            Mat redMask = new Mat();
            Mat redMask2 = new Mat();
            Mat combinedMask = new Mat();
            Core.inRange(roi, lowerBlue, upperBlue, blueMask);
            Core.inRange(roi, lowerGreen, upperGreen, greenMask);
            Core.inRange(roi, lowerRed1, upperRed1, redMask);
            Core.inRange(roi, lowerRed2, upperRed2, redMask2);

            Core.add(blueMask, greenMask, combinedMask);
            Core.add(combinedMask, redMask, combinedMask);
            Core.add(combinedMask, redMask2, combinedMask);

            double pixelSum = Core.sumElems(combinedMask).val[0];
            double contourScore = computeContourArea(combinedMask);
            double score = pixelSum * (1 - CONTOUR_WEIGHT) + contourScore * CONTOUR_WEIGHT;

            blueMask.release();
            greenMask.release();
            redMask.release();
            redMask2.release();
            combinedMask.release();
            roi.release();
            return score;
        }

        private double computeContourArea(Mat mask) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double maxArea = 0;
            for (MatOfPoint contour : contours) {
                maxArea = Math.max(maxArea, Imgproc.contourArea(contour));
                contour.release();
            }
            hierarchy.release();
            return maxArea;
        }

        private double smooth(double previous, double current) {
            if (previous == 0) {
                return current;
            }
            return previous * SCORE_SMOOTHING + current * (1 - SCORE_SMOOTHING);
        }

        public DetectedMotif getCurrentMotif() {
            return currentMotif;
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

    /** AprilTag pipeline focused on backdrop tags 1-3 with ROI filtering */
    private static class AprilTagPipeline extends OpenCvPipeline {
        private final long nativeApriltagPtr;
        private final Mat gray = new Mat();
        private volatile AprilTagDetection latestBackdropDetection;
        private volatile AprilTagDetection latestAnyDetection;

        AprilTagPipeline() {
            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
                    AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
            updateDecimation();
        }

        @Override
        public Mat processFrame(Mat input) {
            Rect roi = clampRect(new Rect(TAG_ROI_X, TAG_ROI_Y, TAG_ROI_WIDTH, TAG_ROI_HEIGHT), input);
            if (roi.width <= 0 || roi.height <= 0) {
                return input;
            }
            Mat cropped = input.submat(roi);
            Imgproc.cvtColor(cropped, gray, Imgproc.COLOR_RGB2GRAY);

            double roiCx = CX - roi.x;
            double roiCy = CY - roi.y;
            ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                    nativeApriltagPtr, gray, TAG_SIZE_METERS, FX, FY, roiCx, roiCy);

            cropped.release();

            latestBackdropDetection = chooseBackdropDetection(detections);
            latestAnyDetection = chooseClosestDetection(detections);

            // Draw ROI for dashboard/preview feedback
            Imgproc.rectangle(input, roi, new Scalar(255, 128, 0), 2);
            if (latestAnyDetection != null) {
                Point center = new Point(latestAnyDetection.center.x + roi.x, latestAnyDetection.center.y + roi.y);
                Imgproc.circle(input, center, 6, new Scalar(0, 255, 255), -1);
                Imgproc.putText(input, String.format("id:%d", latestAnyDetection.id),
                        new Point(center.x - 20, center.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                        new Scalar(0, 255, 255), 2);
            }

            return input;
        }

        private Rect clampRect(Rect roi, Mat input) {
            int x = Math.max(0, roi.x);
            int y = Math.max(0, roi.y);
            int width = Math.min(roi.width, input.width() - x);
            int height = Math.min(roi.height, input.height() - y);
            return new Rect(x, y, Math.max(0, width), Math.max(0, height));
        }

        private AprilTagDetection chooseBackdropDetection(ArrayList<AprilTagDetection> detections) {
            AprilTagDetection best = null;
            for (AprilTagDetection detection : detections) {
                if (detection.id < 1 || detection.id > 3) {
                    continue;
                }
                if (best == null || detection.pose.z < best.pose.z) {
                    best = detection;
                }
            }
            return best;
        }

        private AprilTagDetection chooseClosestDetection(ArrayList<AprilTagDetection> detections) {
            AprilTagDetection best = null;
            for (AprilTagDetection detection : detections) {
                if (best == null || detection.pose.z < best.pose.z) {
                    best = detection;
                }
            }
            return best;
        }

        public BackdropTarget getBestTargetFor(DetectedMotif desired) {
            AprilTagDetection detection = latestBackdropDetection;
            if (detection == null) {
                return null;
            }

            int expectedId = desired == DetectedMotif.MOTIF_A ? 1
                    : desired == DetectedMotif.MOTIF_B ? 2 : 3;
            if (detection.id != expectedId) {
                // still return the seen tag, but note mismatch via heading/offset
            }

            double lateral = detection.pose.x;
            double range = Math.hypot(detection.pose.x, detection.pose.z);
            double headingError = detection.pose.yaw;
            double pixelOffset = detection.center.x - CX;
            return new BackdropTarget(detection.id, range, lateral, headingError, pixelOffset);
        }

        public AprilTagMeaning getLatestTagMeaning() {
            AprilTagDetection detection = latestAnyDetection;
            if (detection == null) {
                return AprilTagMeaning.UNKNOWN;
            }
            return AprilTagMeaning.fromId(detection.id);
        }

        public void updateDecimation() {
            AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, TAG_DECIMATION);
        }
    }
}
