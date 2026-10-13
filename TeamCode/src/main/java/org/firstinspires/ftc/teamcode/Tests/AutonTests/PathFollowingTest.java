package org.firstinspires.ftc.teamcode.Tests.AutonTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Executor;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;
@TeleOp(name = "Path Following Test")
public class PathFollowingTest extends LinearOpMode {
    Chasis chasis;
    Executor executor;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initChasis(this);
        Hardware.initIMU(this);
        Hardware.initFTCDashboard(this);

        chasis = new Chasis(this);
        chasis.setMaxSpeed(1);
        chasis.UseDriver(false); //If using driver: true

        chasis.setMaxAngularSpeed(1);
        chasis.setPosition(new Pose2D(0,0,0));


        executor = new Executor.Builder()
                .setChasis(chasis)
                .build();

        executor.invert(true);
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
            executor.goTo(chasis.getPosition(), new Pose2D(40,0,0));
            executor.goTo(chasis.getPosition(), new Pose2D(0,0,0));

            }


        }

    }
}
