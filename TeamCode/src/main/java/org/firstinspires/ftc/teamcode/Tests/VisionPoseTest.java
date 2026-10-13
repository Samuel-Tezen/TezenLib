package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.Camera;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

@TeleOp(name = "VisionPoseTest", group = "Test")
public class VisionPoseTest extends LinearOpMode {
    Chasis chasis;
    Camera cam;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.initChasis(this);
        Hardware.initFTCDashboard(this);
        chasis = new Chasis(this);
        chasis.UseDriver(true);
        cam = new Camera(this);
        cam.start();
        Pose2D position;
        Pose2D DriverPosition;

        waitForStart();

        while (opModeIsActive()){
            chasis.updatePosition();

            if(cam.isAprilTagDetected()) {
                position = cam.getRobotPoseFromAprilTag();

                telemetry.addData("Distance: ", cam.getDistance());
                telemetry.addData("DistanceX: ", cam.getDistanceX());
            }else{
                position = chasis.getPosition();
            }
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(position, packet);
            DashboardUtil.sendPacket(packet);

            telemetry.addData("distance", cam.getDistance());
            telemetry.update();

        }



    }
}


