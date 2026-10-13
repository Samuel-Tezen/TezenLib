package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    private DcMotorEx motor;
    private boolean direction = true;
    public Encoder(DcMotorEx motor){
        this.motor = motor;
    }
    public double getPosition(){
        return direction?motor.getCurrentPosition(): motor.getCurrentPosition() *-1;
    }
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setDirection(boolean direction){
        this.direction = direction;
    }
}
