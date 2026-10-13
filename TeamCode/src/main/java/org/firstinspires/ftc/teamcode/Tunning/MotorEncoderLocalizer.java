package org.firstinspires.ftc.teamcode.Tunning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;

@TeleOp(name = "MotorEncoderLocalizer")
public class MotorEncoderLocalizer extends LinearOpMode {
    DcMotor FR, FL, BR, BL;
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initFTCDashboard(this);
        this.telemetry = Hardware.ftcDashboard.getTelemetry();
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        initMotors();
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_down) initMotors();
            if(gamepad1.a) FR.setPower(1); else FR.setPower(0);
            if(gamepad1.b) FL.setPower(1); else FL.setPower(0);
            if(gamepad1.x) BR.setPower(1); else BR.setPower(0);
            if(gamepad1.y) BL.setPower(1); else BL.setPower(0);


            telemetry.addData("FR encoder: ", FR.getCurrentPosition());
            telemetry.addData("FL encoder: ", FL.getCurrentPosition());
            telemetry.addData("BR encoder: ", BR.getCurrentPosition());
            telemetry.addData("BL encoder: ", BL.getCurrentPosition());
            telemetry.update();

        }
    }
    public void initMotors(){
        FL.setDirection(DcMotorSimple.Direction.REVERSE);  // Modificar////
        FR.setDirection(DcMotorSimple.Direction.REVERSE);  // si
        BL.setDirection(DcMotorSimple.Direction.REVERSE);  // es/////
        BR.setDirection(DcMotorSimple.Direction.FORWARD );  // necesario


        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
