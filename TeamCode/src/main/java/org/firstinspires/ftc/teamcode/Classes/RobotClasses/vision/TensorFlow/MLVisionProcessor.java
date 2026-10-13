package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.TensorFlow;


import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * VisionProcessor that runs a TFLite SSD detector.
 * - Downscales incoming frame to (inputSize x inputSize) using OpenCV.
 * - Reuses Mat and Bitmap to avoid per-frame allocations.
 * - Throttles processing via frameSkip.
 * - Catches exceptions to avoid reboot loops.
 */
public class MLVisionProcessor implements VisionProcessor {

    public static class Detection2D {
        public final String label;
        public final float score;
        /** Box in camera-frame pixels. */
        public final RectF boxPx;
        public Detection2D(String label, float score, RectF boxPx) {
            this.label = label; this.score = score; this.boxPx = boxPx;
        }
    }

    private final Context appCtx;
    private final String modelPath;
    private final String labelsPath;
    private final int inputSize;
    private final boolean quantized;
    private final int maxDetections;
    private final float scoreThresh;

    private TFLiteSsd detector;

    // Reused buffers to minimize GC and memory churn
    private Mat smallMat;                // inputSize x inputSize
    private Bitmap inputBitmap;          // inputSize x inputSize

    // Latest results for opmode consumption & drawing
    private final AtomicReference<List<Detection2D>> latest = new AtomicReference<>();

    // Simple rendering paints
    private final Paint boxPaint = new Paint();
    private final Paint textPaint = new Paint();

    // Process only 1 out of N frames
    private int frameSkip = 2;
    private int counter = 0;

    public MLVisionProcessor(Context appCtx,
                             String modelPath,
                             String labelsPath,
                             int inputSize,
                             boolean quantized,
                             int maxDetections,
                             float scoreThresh) {
        this.appCtx = appCtx.getApplicationContext();
        this.modelPath = modelPath;
        this.labelsPath = labelsPath;
        this.inputSize = inputSize;
        this.quantized = quantized;
        this.maxDetections = maxDetections;
        this.scoreThresh = scoreThresh;

        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(4f);
        boxPaint.setColor(Color.GREEN);

        textPaint.setTextSize(32f);
        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setColor(Color.WHITE);
    }

    public void setFrameSkip(int n) { this.frameSkip = Math.max(1, n); }

    public void close() {
        try { if (detector != null) detector.close(); } catch (Exception ignored) {}
        try { if (smallMat != null) smallMat.release(); } catch (Exception ignored) {}
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        try {
            detector = new TFLiteSsd(appCtx.getAssets(), modelPath, labelsPath, inputSize, quantized, maxDetections);
        } catch (Exception e) {
            e.printStackTrace();
            detector = null;
        }

        // Preallocate downscaled Mat and Bitmap
        smallMat = new Mat(inputSize, inputSize, CvType.CV_8UC4);
        inputBitmap = Bitmap.createBitmap(inputSize, inputSize, Bitmap.Config.ARGB_8888);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (detector == null) return null;

        counter = (counter + 1) % frameSkip;
        if (counter != 0) return null; // throttle

        try {
            // 1) Downscale BEFORE conversion to Bitmap to save memory
            Imgproc.resize(frame, smallMat, new Size(inputSize, inputSize), 0, 0, Imgproc.INTER_AREA);

            // 2) Convert Mat -> Bitmap (reused)
            Utils.matToBitmap(smallMat, inputBitmap);

            // 3) Run inference (fast path - pre-resized)
            List<TFLiteSsd.Detection> raw = detector.detectPreResized(inputBitmap, scoreThresh);

            // 4) Convert normalized detections to pixel coords of the ORIGINAL frame
            int W = frame.width();
            int H = frame.height();
            List<Detection2D> out = new ArrayList<>(raw.size());
            for (TFLiteSsd.Detection d : raw) {
                RectF px = new RectF(
                        d.box01.left   * W,
                        d.box01.top    * H,
                        d.box01.right  * W,
                        d.box01.bottom * H
                );
                out.add(new Detection2D(d.label, d.score, px));
            }
            latest.set(out);

        } catch (Throwable t) {
            // Safety net: never crash/reboot loop due to ML exceptions
            t.printStackTrace();
            latest.set(java.util.Collections.emptyList());
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        List<Detection2D> dets = latest.get();
        if (dets == null || dets.isEmpty()) return;

        for (Detection2D d : dets) {
            RectF scaled = new RectF(
                    d.boxPx.left   * scaleBmpPxToCanvasPx,
                    d.boxPx.top    * scaleBmpPxToCanvasPx,
                    d.boxPx.right  * scaleBmpPxToCanvasPx,
                    d.boxPx.bottom * scaleBmpPxToCanvasPx
            );
            canvas.drawRect(scaled, boxPaint);
            canvas.drawText(d.label + String.format(" (%.2f)", d.score),
                    scaled.left, Math.max(0, scaled.top - 6), textPaint);
        }
    }

    /** Thread-safe getter for latest detections. */
    public List<Detection2D> getLatestDetections() {
        List<Detection2D> dets = latest.get();
        return dets == null ? java.util.Collections.emptyList() : dets;
    }
}
