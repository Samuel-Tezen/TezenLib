package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.TensorFlow;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.RectF;

//import org.tensorflow.lite.Interpreter;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Lightweight wrapper for SSD-like TFLite object detection.
 * - Loads model from assets via InputStream (no mmap) to avoid packaging/compression issues.
 * - Uses 1 thread by default for stability on Control Hub.
 * - Provides detect() and detectPreResized() to avoid per-frame scaling.
 */
public class TFLiteSsd {

    public static class Detection {
        public final String label;
        public final float score;
        /** Normalized [0..1] box in order [left, top, right, bottom]. */
        public final RectF box01;
        public Detection(String label, float score, RectF box01) {
            this.label = label; this.score = score; this.box01 = box01;
        }
    }

 //   private final Interpreter tflite;
    private final int inputSize;
    private final boolean quantized;
    private final int maxDetections;

    private final ByteBuffer imgBuffer;
    private int[] pixelBuffer; // reused for bitmapToBuffer
    private final List<String> labels;

    public TFLiteSsd(AssetManager am,
                     String modelPath,
                     String labelsPath,
                     int inputSize,
                     boolean quantized,
                     int maxDetections) throws IOException {

        this.inputSize = inputSize;
        this.quantized = quantized;
        this.maxDetections = maxDetections;

        // Load model bytes via InputStream and copy into a direct ByteBuffer
        ByteBuffer modelBuffer = loadModelFromAssets(am, modelPath);

      //  Interpreter.Options opts = new Interpreter.Options();
        // Safer on Control Hub / low-memory devices
     //   opts.setNumThreads(1);

      //  this.tflite = new Interpreter(modelBuffer, opts);

        int bytesPerChannel = quantized ? 1 : 4;
        this.imgBuffer = ByteBuffer
                .allocateDirect(inputSize * inputSize * 3 * bytesPerChannel)
                .order(ByteOrder.nativeOrder());

        this.labels = loadLabels(am, labelsPath);
        this.pixelBuffer = new int[inputSize * inputSize];
    }

    public void close() {
   //     try { tflite.close(); } catch (Exception ignored) {}
    }

    private static ByteBuffer loadModelFromAssets(AssetManager am, String path) throws IOException {
        try (java.io.InputStream is = am.open(path)) {
            byte[] bytes = readAllBytes(is);
            ByteBuffer bb = ByteBuffer.allocateDirect(bytes.length);
            bb.order(ByteOrder.nativeOrder());
            bb.put(bytes);
            bb.rewind();
            return bb;
        }
    }

    private static byte[] readAllBytes(java.io.InputStream is) throws IOException {
        java.io.ByteArrayOutputStream bos = new java.io.ByteArrayOutputStream();
        byte[] buf = new byte[16 * 1024];
        int r;
        while ((r = is.read(buf)) != -1) bos.write(buf, 0, r);
        return bos.toByteArray();
    }

    private static List<String> loadLabels(AssetManager am, String path) throws IOException {
        List<String> out = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new InputStreamReader(am.open(path)))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.isEmpty()) out.add(line);
            }
        }
        return out;
    }

    /** Converts a Bitmap (inputSize x inputSize) into the input tensor buffer. */
    private void bitmapToBuffer(Bitmap bmp) {
        imgBuffer.rewind();
        // Reuse pixelBuffer to avoid allocations per frame
        if (pixelBuffer == null || pixelBuffer.length != inputSize * inputSize) {
            pixelBuffer = new int[inputSize * inputSize];
        }
        bmp.getPixels(pixelBuffer, 0, inputSize, 0, 0, inputSize, inputSize);
        if (quantized) {
            for (int v : pixelBuffer) {
                imgBuffer.put((byte) ((v >> 16) & 0xFF)); // R
                imgBuffer.put((byte) ((v >> 8)  & 0xFF)); // G
                imgBuffer.put((byte) (v & 0xFF));         // B
            }
        } else {
            for (int v : pixelBuffer) {
                imgBuffer.putFloat(((v >> 16) & 0xFF) / 255f);
                imgBuffer.putFloat(((v >> 8)  & 0xFF) / 255f);
                imgBuffer.putFloat((v & 0xFF) / 255f);
            }
        }
    }

    /** Full detect path (will create a scaled copy). Prefer detectPreResized() for performance. */
    public List<Detection> detect(Bitmap src, float scoreThresh) {
        Bitmap resized = Bitmap.createScaledBitmap(src, inputSize, inputSize, true);
        return detectPreResized(resized, scoreThresh);
    }

    /**
     * Fast path: caller provides a Bitmap already resized to (inputSize x inputSize).
     * This avoids allocating a large intermediate Bitmap every frame.
     */
    public List<Detection> detectPreResized(Bitmap resized, float scoreThresh) {
        if (resized.getWidth() != inputSize || resized.getHeight() != inputSize) {
            // As a safety, fall back to slow path if sizes mismatch
            return detect(resized, scoreThresh);
        }

        // Prepare input tensor
        bitmapToBuffer(resized);

        // Typical SSD TFLite outputs: boxes, classes, scores, numDetections
        float[][][] boxes = new float[1][maxDetections][4];   // [ymin, xmin, ymax, xmax] in 0..1
        float[][] classes = new float[1][maxDetections];      // class indices
        float[][] scores  = new float[1][maxDetections];      // confidences
        float[] num       = new float[1];

        Object[] inputs = { imgBuffer };
        Map<Integer, Object> outputs = new HashMap<>();
        outputs.put(0, boxes);
        outputs.put(1, classes);
        outputs.put(2, scores);
        outputs.put(3, num);

        // Inference
      //  tflite.runForMultipleInputsOutputs(inputs, outputs);

        int n = Math.min(maxDetections, (int) (num[0] <= 0 ? maxDetections : num[0]));
        List<Detection> result = new ArrayList<>(n);

        for (int i = 0; i < n; i++) {
            float sc = scores[0][i];
            if (sc < scoreThresh) continue;

            float top = boxes[0][i][0];
            float left = boxes[0][i][1];
            float bottom = boxes[0][i][2];
            float right = boxes[0][i][3];

            int idx = (int) classes[0][i];
            String label = (idx >= 0 && idx < labels.size()) ? labels.get(idx) : ("id:" + idx);

            // Convert to [left, top, right, bottom] normalized
            result.add(new Detection(label, sc, new RectF(left, top, right, bottom)));
        }
        return result;
    }
}

