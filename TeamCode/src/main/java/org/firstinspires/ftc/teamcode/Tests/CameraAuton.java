package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Executor;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.Camera;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.TensorFlow.MLVisionProcessor;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.List;


@TeleOp(name = "Camera Auton")
public class CameraAuton extends LinearOpMode {
    Chasis chasis;
    Pose2D position;
    Camera camera;
    Executor executor;
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initChasis(this);
        Hardware.initIMU(this);
        Hardware.initFTCDashboard(this);

        chasis = new Chasis(this);
        chasis.setMaxSpeed(0.6);
        chasis.setMaxAngularSpeed(0.3);
        camera = new Camera(this);
        camera.start();

        executor = new Executor.Builder()
                .setChasis(chasis)
                .setCamera(camera)
                .build();

        waitForStart();
        while (opModeIsActive()) {
            chasis.updatePosition();
            chasis.power(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            position = chasis.getPosition();


            if (gamepad1.a) {
                chasis.resetPosition();
            }
            if(gamepad1.x){
                executor.alignToColor("red", 7, new Pose2D(-16, -10,-45));
            }

            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(position, packet);
            if(camera.getColorPose("red", 7,position) != null) {
                DashboardUtil.drawPoint(camera.getColorPose("red", 7, position), packet, "green");
            }
            DashboardUtil.sendPacket(packet);

            List<MLVisionProcessor.Detection2D> dets = camera.getMLDetections();
            telemetry.addData("ML Detections", dets.size());
            for (int i = 0; i < dets.size(); i++) {
                MLVisionProcessor.Detection2D d = dets.get(i);
                telemetry.addData("#"+i, "%s (%.2f) [%.0f,%.0f,%.0f,%.0f]",
                        d.label, d.score, d.boxPx.left, d.boxPx.top, d.boxPx.right, d.boxPx.bottom);
            }
            telemetry.addData("distance: ",camera.getColorDistance("red", 7));
            telemetry.addData("strafe: ",camera.getColorStrafe("red", 7));
            telemetry.update();


        }
        camera.stop();

    }

}
