package org.firstinspires.ftc.teamcode.Classes.ToolClasses.MotorControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double kp;
    private final double ki;
    private final double kd;
    private double areaIntegral = 10;

    private double integralSum = 0.0000000000001;

    private double lasterror = 0.000000000000001;


    private final ElapsedTime timer1 = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void reset(){
        integralSum = 0;
        lasterror = 0;
    }

    public void setAreaIntegral(double areaIntegral) {
        this.areaIntegral = areaIntegral;
    }


    public double getOutput(double error, boolean direccion) {
        if(Math.abs(error)<areaIntegral) {
            integralSum += error * timer1.seconds();
        }
        double derivative = (error - lasterror) / timer1.seconds();

        lasterror = error;

        timer1.reset();
        double output = (error * this.kp)
                + ( integralSum * this.ki)
                + ( derivative * this.kd);

        if (direccion) {
            return output;
        } else {
            return output*-1;
        }
    }




}