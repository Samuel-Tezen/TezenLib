package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Executor;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.BezierCurveGenerator;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.ArrayList;

@Config
@TeleOp(name = "AutonBezierMotionProfile", group = "TEST")
public class AutonBezierMotionProfile extends LinearOpMode {
    public  ArrayList<Pose2D> waypoints = new ArrayList<>();
    public  ArrayList<Pose2D> waypoints2= new ArrayList<>();

    public static int resolution = 40;
    public static double  totalTime = 40;

    public static double[] Y = {0,10,20,30};
    public static double[] X = {0,10,-10,0};
    public static double[] heading ={90,-90,0,0};

    public static double[] Y2 = {30,20,10,0};
    public static double[] X2 = {0,-10,10,0};
    public static double[] heading2 ={0,0,0,0};
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initChasis(this);
        Hardware.initIMU(this);
        Hardware.initFTCDashboard(this);


        Chasis chasis = new Chasis(this);
        chasis.setPosition(new Pose2D(10,0,20));
        Executor executor = new Executor.Builder()
                .setChasis(chasis).build();

        // Reset odometría
        chasis.resetPosition();

        // Trazo base de puntos clave
        waypoints.add(new Pose2D(Y[0], X[0], heading[0]));
        waypoints.add(new Pose2D(Y[1], X[1], heading[1]));
        waypoints.add(new Pose2D(Y[2], X[2], heading[2]));
        waypoints.add(new Pose2D(Y[3], X[3], heading[3]));

        waypoints2.add(new Pose2D(Y2[0], X2[0], heading2[0]));
        waypoints2.add(new Pose2D(Y2[1], X2[1], heading2[1]));
        waypoints2.add(new Pose2D(Y2[2], X2[2], heading2[2]));
        waypoints2.add(new Pose2D(Y2[3], X2[3], heading2[3]));

        ArrayList<Pose2D> curve = BezierCurveGenerator.generateCubicBezierPath(waypoints, resolution, totalTime);
        ArrayList<Pose2D> curve2 = BezierCurveGenerator.generateCubicBezierPath(waypoints2, resolution, totalTime);

        // Visualización antes del start
        while (!isStarted() && !isStopRequested()) {
            chasis.updatePosition();
            Pose2D pose = chasis.getPosition();

            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(pose, packet);
            DashboardUtil.drawPath(curve, packet);
            DashboardUtil.drawPath(curve2, packet);
            DashboardUtil.sendPacket(packet);

            telemetry.addData("Pose X", pose.x);
            telemetry.addData("Pose Y", pose.y);
            telemetry.addData("Theta", pose.theta);
            telemetry.update();
        }

        // Start
        waitForStart();

        // Generar curva cúbica Bézier

        // Ejecutar trayectoria con motion profiling
        executor.followPath(curve); // in/s, in/s²
        executor.followPath(curve2); // in/s, in/s²

        // Stop final
        chasis.power(0, 0, 0);
    }
}
