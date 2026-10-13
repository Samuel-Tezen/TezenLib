package org.firstinspires.ftc.teamcode.Tunning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Executor;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

@Autonomous(name = "PIDTunning", group = "Tunning")
public class PIDTunning extends LinearOpMode {
    Chasis chasis;
    Executor TE;
    @Override
    public void runOpMode() throws InterruptedException {


        OdometryConstants.Y_OffSet = 0;
        OdometryConstants.X_OffSet = 0;
        Hardware.initAll(this);

        chasis = new Chasis(this);
        chasis.UseDriver(true); //If using driver: true
        chasis.setPinpointPosition(OdometryConstants.X_OffSet, OdometryConstants.Y_OffSet, 0);
        TE = new Executor.Builder()
                .setChasis(chasis).build();
        while (!isStarted() && !isStopRequested()) {
            chasis.updatePosition();
            Pose2D pose = chasis.getPosition();

            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(pose, packet);
            DashboardUtil.sendPacket(packet);

            telemetry.addData("Pose X", pose.x);
            telemetry.addData("Pose Y", pose.y);
            telemetry.addData("Theta", pose.theta);
            telemetry.update();

        }
        waitForStart();
        while (opModeIsActive()){
            TE.goTo(chasis.getPosition(), new Pose2D(0,0,0));
            TE.goTo(chasis.getPosition(), new Pose2D(40,0,0));
        }
    }
}
