package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.ArrayList;

public class DashboardUtil {

    private static final FtcDashboard dashboard = Hardware.ftcDashboard;


    public static void drawPoint(Pose2D point, TelemetryPacket packet, String color) {
        Canvas canvas = packet.fieldOverlay();

        double radius = 1.5; // pulgadas

        canvas.setFill(color);
        canvas.fillCircle(-point.y, point.x, radius);
    }

    public static void drawRobot(Pose2D pose, TelemetryPacket packet) {
        Canvas field = packet.fieldOverlay();
        double robotRadius = 7.75; // en pulgadas
        double headingRad = Math.toRadians(pose.theta + 90);
        // Posición
        field.setStroke("red");
        field.strokeCircle(-pose.y, pose.x, robotRadius);

        // Flecha de orientación
        double arrowLength = 12.0;
        double endX = pose.x + arrowLength * Math.cos(headingRad);
        double endY = pose.y + arrowLength * Math.sin(headingRad);
        field.strokeLine(-pose.y, pose.x, -endY, endX);
    }

    public static void drawPath(ArrayList<Pose2D> path, TelemetryPacket packet) {
        Canvas field = packet.fieldOverlay();
        field.setStroke("green");
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2D p1 = path.get(i);
            Pose2D p2 = path.get(i + 1);
            field.strokeLine(-p1.y, p1.x, -p2.y, p2.x);
        }

        for (Pose2D point : path) {
            field.fillCircle(-point.y,point.x, 1);
        }
    }

    public static void sendPacket(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
    }
    public static void exportTrajectory(ArrayList<Pose2D> path, String name) {
        StringBuilder output = new StringBuilder();
        output.append("ArrayList&lt;Pose2D&gt; ").append(name).append(" = ArrayList.of(\n");

        for (int i = 0; i < path.size(); i++) {
            Pose2D p = path.get(i);
            output.append(String.format(name+".add (new Pose2D(%.2f, %.2f, %.2f, %.2f))%s\n",
                    p.y, p.x, p.theta, p.time,
                    (i < path.size() - 1) ? "," : ""));
        }

        output.append(");");

        // Puedes imprimir esto en consola o en telemetry
        System.out.println(output);

        // O enviarlo como multiline telemetry
        for (String line : output.toString().split("\n")) {
            dashboard.getTelemetry().addLine(line);
        }
        dashboard.getTelemetry().update();

    }
}

