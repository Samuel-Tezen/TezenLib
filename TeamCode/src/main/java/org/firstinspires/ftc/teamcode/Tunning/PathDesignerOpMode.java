package org.firstinspires.ftc.teamcode.Tunning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.BezierCurveGenerator;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.ArrayList;

@TeleOp(name = "PathDesigner (Gamepad)")
public class PathDesignerOpMode extends LinearOpMode {

    private final ArrayList<Pose2D> waypoints = new ArrayList<>();
    private Pose2D cursor = new Pose2D(OdometryConstants.Y_OffSet, OdometryConstants.X_OffSet, OdometryConstants.Angular_Offset);
    private boolean wasA = false, wasB = false, wasStart = false;
    private boolean finished = false;

    public static ArrayList<Pose2D> lastDrawed = new ArrayList<>();
    @Override
    public void runOpMode() {

        waitForStart();
        Hardware.initFTCDashboard(this);
        while (opModeIsActive()) {
            updateCursor();

            boolean a = gamepad1.aWasPressed();
            boolean b = gamepad1.bWasPressed();
            boolean start = gamepad1.start;

            if (a && !wasA && !finished) {
                waypoints.add(new Pose2D(cursor.y, cursor.x, cursor.theta));
                sleep(200);
            }

            if (b && !wasB && !finished && !waypoints.isEmpty()) {
                waypoints.remove(waypoints.size() - 1);
                sleep(200);

            }

            if (start && !wasStart) {
                finished = true;
            }

            wasA = a;
            wasB = b;
            wasStart = start;

            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(cursor, packet);

            for (Pose2D p : waypoints) {
                DashboardUtil.drawPoint(p, packet, "green");
            }

            if (finished && waypoints.size() >= 2) {
                ArrayList<Pose2D> curve = BezierCurveGenerator.generateCubicBezierPath(waypoints, 40, 4.0);
                DashboardUtil.exportTrajectory(waypoints, "exported");
                DashboardUtil.drawPath(curve, packet);
                lastDrawed = curve;
            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.addData("Cursor", "(%.1f, %.1f)", cursor.x, cursor.y);
            telemetry.addData("Puntos", waypoints.size());
            telemetry.addLine("A: Agregar punto | B: Borrar último | START: Terminar");
            telemetry.update();
        }
    }
    private void updateCursor() {
        double moveSpeed = 0.001;
        double angularSpeed = 0.007;

        double dx = -gamepad1.left_stick_x * moveSpeed;
        double dy = gamepad1.left_stick_y * moveSpeed;
        double dt = -gamepad1.right_stick_x * angularSpeed;

        cursor = new Pose2D(cursor.y + dy, cursor.x + dx, cursor.theta+dt);
    }


}
