package org.firstinspires.ftc.teamcode.Tunning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "ServoLocalizer")
public class ServoLocalizer extends LinearOpMode {
    Servo MunecaL, MunecaR, Muneca, Garra, Brazo;
    @Override
    public void runOpMode() throws InterruptedException {
        MunecaL = hardwareMap.get(Servo.class, "WristL");
        MunecaR = hardwareMap.get(Servo.class, "WristR");

        Muneca = hardwareMap.get(Servo.class, "Wrist");

        Garra = hardwareMap.get(Servo.class, "Grip");

        Brazo = hardwareMap.get(Servo.class, "Arms");

        MunecaL.setDirection(Servo.Direction.FORWARD);
        MunecaR.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                MunecaL.setPosition(1);
                sleep(1000);
                MunecaL.setPosition(0);
            }
            if(gamepad1.b){
                MunecaR.setPosition(1);
                sleep(1000);
                MunecaR.setPosition(0);
            }
            if(gamepad1.x){
                Garra.setPosition(1);
                sleep(1000);
                Garra.setPosition(0);
            }
            if(gamepad1.y){
                Muneca.setPosition(1);
                sleep(1000);
                Muneca.setPosition(0);
            }
            if(gamepad1.dpad_up){
                Brazo.setPosition(1);
                sleep(1000);
                Brazo.setPosition(0);
            }

        }
    }


}
