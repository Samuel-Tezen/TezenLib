package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Huskylens calibration")
public class HuskyLensCalibration extends LinearOpMode {
    HuskyLens huskyLens;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initFTCDashboard(this);
        this.telemetry = Hardware.ftcDashboard.getTelemetry();
        huskyLens = hardwareMap.get(HuskyLens.class,"huskylens");
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();


        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        while (opModeIsActive()){
            rateLimit.reset();
            HuskyLens.Block[] blocks = huskyLens.blocks();

            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Width", blocks[i].width);

            }

            telemetry.update();}
    }
}
