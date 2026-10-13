package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision;

import static org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.CameraConstants.CameraHeight;
import static org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.CameraConstants.FocalLength;
import static org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.CamMaths.estimateObjectPose2;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.CamMaths;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ColorProcessor implements VisionProcessor {

    private volatile Point blueCenter = new Point(-1, -1);
    private volatile Point redCenter = new Point(-1, -1);
    private volatile Point yellowCenter = new Point(-1, -1);

    private volatile Rect redRect = null;
    private volatile Rect blueRect = null;
    private volatile Rect yellowRect = null;



    private Mat displayMat = new Mat(); // lo que se verá en pantalla
    private int frameW = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No necesitas hacer nada aquí por ahora
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        input.copyTo(displayMat);
        frameW = input.width();


        Mat hsv = new Mat();
        Imgproc.cvtColor(displayMat, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 0);


        // ---------------- AZUL ----------------
        blueCenter = detectCenterColor(hsv, displayMat, new Scalar(100, 100, 50), new Scalar(130, 255, 255), new Scalar(0, 0, 255));
        blueRect = detectRectColor(hsv, displayMat, new Scalar(100, 100, 50), new Scalar(130, 255, 255), new Scalar(0, 0, 255));

        // ---------------- AMARILLO ----------------
        yellowCenter = detectCenterColor(hsv, displayMat, new Scalar(20, 100, 100), new Scalar(30, 255, 255), new Scalar(0, 255, 255));
        yellowRect = detectRectColor(hsv, displayMat, new Scalar(20, 100, 100), new Scalar(30, 255, 255), new Scalar(0, 255, 255));

        // ---------------- ROJO ----------------
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat redMask = new Mat();

        Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(180, 255, 255), mask2);
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, redMask);
        Imgproc.GaussianBlur(redMask, redMask, new Size(5, 5), 0);

        ArrayList<MatOfPoint> redContours = new ArrayList<>();
        Mat redHierarchy = new Mat();
        Imgproc.findContours(redMask, redContours, redHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double redMaxArea = 0;
        Rect redBestRect = null;
        for (MatOfPoint contour : redContours) {
            double area = Imgproc.contourArea(contour);
            if (area > redMaxArea) {
                redMaxArea = area;
                redBestRect = Imgproc.boundingRect(contour);
            }
        }

        if (redBestRect != null && redMaxArea > 500) {
            redRect = redBestRect;
            redCenter = new Point(redBestRect.x + redBestRect.width / 2.0, redBestRect.y + redBestRect.height / 2.0);
            Imgproc.rectangle(displayMat, redBestRect, new Scalar(255, 0, 0), 2);
            Imgproc.circle(displayMat, redCenter, 5, new Scalar(255, 0, 0), -1);
        } else {
            redCenter = new Point(-1, -1);
        }

        return null;
    }

    private Point detectCenterColor(Mat hsv, Mat input, Scalar lower, Scalar upper, Scalar drawColor) {
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
            }
        }

        if (bestRect != null && maxArea > 500) {
            Point center = new Point(bestRect.x + bestRect.width / 2.0, bestRect.y + bestRect.height / 2.0);
            Imgproc.rectangle(input, bestRect, drawColor, 2);
            Imgproc.circle(input, center, 5, drawColor, -1);
            return center;
        }

        return new Point(-1, -1);
    }
    private Rect detectRectColor(Mat hsv, Mat input, Scalar lower, Scalar upper, Scalar drawColor) {
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
            }
        }

        if (bestRect != null && maxArea > 500) {
            Point center = new Point(bestRect.x + bestRect.width / 2.0, bestRect.y + bestRect.height / 2.0);
            Imgproc.rectangle(input, bestRect, drawColor, 2);
            Imgproc.circle(input, center, 5, drawColor, -1);
            return bestRect;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (!displayMat.empty()) {
            Bitmap bmp = Bitmap.createBitmap(displayMat.cols(), displayMat.rows(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(displayMat, bmp);
            canvas.drawBitmap(bmp, 0, 0, null);
        }
    }

    // Getters
    public Point getBlueCenter() { return blueCenter; }
    public Point getRedCenter() { return redCenter; }
    public Point getYellowCenter() { return yellowCenter; }

    public Rect getRedRect() {
        return redRect;
    }
    public Rect getBlueRect() {
        return blueRect;
    }
    public Rect getYellowRect() {
        return yellowRect;
    }

    public double getRedDistance(double height){
        if(getRedRect() == null) return 0;
        double rawDistance = (height * FocalLength) /getRedRect().height;
        return Math.sqrt((rawDistance*rawDistance)-(CameraHeight*CameraHeight));
    }
    public double getBlueDistance(double height){
        double rawDistance = (height * FocalLength) /getBlueRect().height;
        return Math.sqrt((rawDistance*rawDistance)-(CameraHeight*CameraHeight));

    }
    public double getYellowDistance(double height){
        double rawDistance = (height * FocalLength) /getYellowRect().height;
        return Math.sqrt((rawDistance*rawDistance)-(CameraHeight*CameraHeight));    }


    public double getRedDistanceX(double height) {
        if (redCenter.x == -1 || redRect == null) return 0;
        return CamMaths.estimateXOffsetFromCenter(redCenter, frameW, getRedDistance(height));


    }

    public double getBlueDistanceX(double height) {
        if (blueCenter.x == -1 || blueRect == null) return 0;

        return CamMaths.estimateXOffsetFromCenter(blueCenter, frameW, getRedDistance(height));
    }

    public double getYellowDistanceX(double height) {
        if (yellowCenter.x == -1 || yellowRect == null) return 0;

        return CamMaths.estimateXOffsetFromCenter(yellowCenter, frameW, getRedDistance(height));
    }

    public enum Color{
        RED,
        BLUE,
        YELLOW
    }
    public double getColorDistance(Color c, double height){
        switch (c){
            case RED:
                return getRedDistance(height);
            case BLUE:
                return getBlueDistance(height);
            case YELLOW:
                return getYellowDistance(height);

        }
        return -1;
    }
    public double getColorStrafe(Color c, double height){
        switch (c){
            case RED:
                return getRedDistanceX(height);
            case BLUE:
                return getBlueDistanceX(height);
            case YELLOW:
                return getYellowDistanceX(height);
        }
        return -1;
    }

    //Calibrar fov a 30 pulgadas
    public double getFov(){
        return (double) (getRedRect().height * 30) /7;
    }



    public Pose2D getObjectFieldPose(String color, double objectHeight, Pose2D currentPose){
        double distanceZ = 0;
        double distanceX = 0;

        switch (color.toLowerCase()) {
            case "red":
                distanceZ = getRedDistance(objectHeight);
                distanceX = getRedDistanceX(objectHeight);
                break;
            case "blue":
                distanceZ = getBlueDistance(objectHeight);
                distanceX = getBlueDistanceX(objectHeight);
                break;
            case "yellow":
                distanceZ = getYellowDistance(objectHeight);
                distanceX = getYellowDistanceX(objectHeight);
                break;

        }

        return estimateObjectPose2(currentPose, distanceZ,distanceX);
    }
}
