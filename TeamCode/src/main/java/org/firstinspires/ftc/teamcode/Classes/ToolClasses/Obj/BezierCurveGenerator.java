package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj;
import java.util.ArrayList;

public class BezierCurveGenerator {

    public static ArrayList<Pose2D> generateQuadraticBezierTimed(Pose2D p0, Pose2D p1, Pose2D p2, int resolution, double totalTime) {
        ArrayList<Pose2D> result = new ArrayList<>();

        for (int i = 0; i <= resolution; i++) {
            double t = i / (double) resolution;

            double x = Math.pow(1 - t, 2) * p0.x + 2 * (1 - t) * t * p1.x + Math.pow(t, 2) * p2.x;
            double y = Math.pow(1 - t, 2) * p0.y + 2 * (1 - t) * t * p1.y + Math.pow(t, 2) * p2.y;

            double theta = (1 - t) * p0.theta + t * p2.theta;

            double timeAtPoint = t * totalTime;

            result.add(new Pose2D(y, x, theta, timeAtPoint));
        }

        return result;
    }
    public static ArrayList<Pose2D> generateBezierPath(ArrayList<Pose2D> points, int resolution, double totalTime) {
        ArrayList<Pose2D> fullPath = new ArrayList<>();

        if (points.size() < 3) return fullPath; // necesita al menos 3

        int segments = points.size() - 2;
        double timePerSegment = totalTime / segments;

        for (int i = 0; i < segments; i++) {
            Pose2D p0 = points.get(i);
            Pose2D p1 = points.get(i + 1); // control
            Pose2D p2 = points.get(i + 2);

            ArrayList<Pose2D> segment = generateQuadraticBezierTimed(p0, p1, p2, resolution, timePerSegment);

            // Evitar puntos duplicados (repetición en uniones)
            if (i > 0) segment.remove(0);

            fullPath.addAll(segment);
        }

        return fullPath;
    }
    private static ArrayList<Pose2D> generateCubicSegment(Pose2D p0, Pose2D p1, Pose2D p2, Pose2D p3, int resolution, double segmentTime) {
        ArrayList<Pose2D> result = new ArrayList<>();

        for (int i = 0; i <= resolution; i++) {
            double t = i / (double) resolution;

            double x = Math.pow(1 - t, 3) * p0.x +
                    3 * Math.pow(1 - t, 2) * t * p1.x +
                    3 * (1 - t) * Math.pow(t, 2) * p2.x +
                    Math.pow(t, 3) * p3.x;

            double y = Math.pow(1 - t, 3) * p0.y +
                    3 * Math.pow(1 - t, 2) * t * p1.y +
                    3 * (1 - t) * Math.pow(t, 2) * p2.y +
                    Math.pow(t, 3) * p3.y;

            double theta = p3.theta;
            double time = t * segmentTime;

            result.add(new Pose2D(y, x, theta, time));
        }

        return result;
    }
    public static ArrayList<Pose2D> generateCubicBezierPath(ArrayList<Pose2D> waypoints, int resolution, double totalTime) {
        ArrayList<Pose2D> fullPath = new ArrayList<>();
        if (waypoints.size() < 2) return fullPath;

        int segments = waypoints.size() - 1;
        double timePerSegment = totalTime / segments;

        for (int i = 0; i < segments; i++) {
            Pose2D p0 = waypoints.get(i);
            Pose2D p3 = waypoints.get(i + 1);

            // Puntos anteriores y siguientes seguros
            Pose2D prev = (i > 0) ? waypoints.get(i - 1) : p0;
            Pose2D next = (i + 2 < waypoints.size()) ? waypoints.get(i + 2) : p3;

            // Calcular puntos de control usando técnica de centrado
            Pose2D p1 = new Pose2D(
                    p0.y + (p3.y - prev.y) / 6.0,
                    p0.x + (p3.x - prev.x) / 6.0,
                    p3.theta
            );

            Pose2D p2 = new Pose2D(
                    p3.y - (next.y - p0.y) / 6.0,
                    p3.x - (next.x - p0.x) / 6.0,
                    p3.theta
            );

            // Generar segmento
            ArrayList<Pose2D> segment = generateCubicSegment(p0, p1, p2, p3, resolution, timePerSegment);

            if (i > 0) segment.remove(0); // evitar duplicado entre uniones
            if (i > 0 && !segment.isEmpty()) {
                segment.remove(0);
            }

            fullPath.addAll(segment);
        }

        return fullPath;
    }
    public static double interpolateAngle(double theta1, double theta2, double t) {
        return theta1 + (theta2 - theta1) * t;
    }

}
