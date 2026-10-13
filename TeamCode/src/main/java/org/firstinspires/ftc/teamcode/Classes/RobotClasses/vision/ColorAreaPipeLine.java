package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorAreaPipeLine extends OpenCvPipeline{


    private volatile Point blueCenter = new Point(-1, -1);
    private volatile Point redCenter = new Point(-1, -1);
    private volatile Point yellowCenter = new Point(-1, -1);

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Detectar azul y amarillo normalmente
        blueCenter = detectColor(hsv, input, new Scalar(100, 100, 50), new Scalar(130, 255, 255), new Scalar(0, 0, 255)); // Azul
        yellowCenter = detectColor(hsv, input, new Scalar(20, 100, 100), new Scalar(30, 255, 255), new Scalar(0, 255, 255)); // Amarillo

        // Detectar rojo con doble rango HSV
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat redMask = new Mat();

        Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(180, 255, 255), mask2);
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, redMask);
        Imgproc.GaussianBlur(redMask, redMask, new Size(5, 5), 0);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                Rect rect = Imgproc.boundingRect(contour);
                maxArea = area;
                bestRect = rect;
            }
        }

        if (bestRect != null && maxArea > 500) {
            redCenter = new Point(bestRect.x + bestRect.width / 2.0, bestRect.y + bestRect.height / 2.0);
            Imgproc.rectangle(input, bestRect, new Scalar(255, 0, 0), 2);
            Imgproc.circle(input, redCenter, 5, new Scalar(255, 0, 0), -1);
        } else {
            redCenter = new Point(-1, -1);
        }

        return input;
    }

    private Point detectColor(Mat hsv, Mat input, Scalar lower, Scalar upper, Scalar drawColor) {
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
                Rect rect = Imgproc.boundingRect(contour);
                maxArea = area;
                bestRect = rect;
            }
        }

        if (bestRect != null && maxArea > 500) {
            Point center = new Point(bestRect.x + bestRect.width / 2.0, bestRect.y + bestRect.height / 2.0);
            Imgproc.rectangle(input, bestRect, drawColor, 2);
            Imgproc.circle(input, center, 5, drawColor, -1);
            return center;
        }

        return new Point(-1, -1); // No encontrado
    }



    public Point getBlueCenter() { return blueCenter; }
    public Point getRedCenter() { return redCenter; }
    public Point getYellowCenter() { return yellowCenter; }


}


