package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Chasis tester encoderRevvvvv")
public class REVChasisTest extends LinearOpMode {
    Chasis chasis;
    Pose2D position;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initChasis(this);
        Hardware.initIMU(this);
        Hardware.initFTCDashboard(this);
        this.telemetry = Hardware.ftcDashboard.getTelemetry();

        chasis = new Chasis(this);
        chasis.setMaxSpeed(1);
        chasis.setMaxAngularSpeed(1);

        waitForStart();
        while (opModeIsActive()) {

            chasis.updatePosition();
            chasis.power(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            position = chasis.getPosition();

            if(gamepad1.b){
            }else{

            }
            if (gamepad1.a) {
                chasis.resetPosition();
            }

            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(position, packet);
            DashboardUtil.sendPacket(packet);

            telemetry.addData("Y:", position.y);
            telemetry.addData("X:", position.x);
            telemetry.addData("Theta:", position.theta);
            telemetry.update();


        }
    }

}
