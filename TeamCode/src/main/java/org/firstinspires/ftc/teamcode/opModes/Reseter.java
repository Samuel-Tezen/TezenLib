package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
@TeleOp(name ="Reseter")

public class Reseter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initAll(this);
    }
}
