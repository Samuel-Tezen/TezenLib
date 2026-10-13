package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.ArrayList;

public class Zone{
    private Telemetry telemetry;
    public String name;
    private ArrayList<String> Captions = new ArrayList<>();
    private ArrayList<DebugClass.Double> Data = new ArrayList<>();


    public Zone(Telemetry telemetry, String name){
        this.telemetry = telemetry;
        this.name = name;
    }
    public void addData(String caption, double data){
        Captions.add(caption);
        Data.add(new DebugClass.Double(data));
    }

    protected void drawRobot(Pose2D pose){
        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawRobot(pose, packet);
        DashboardUtil.sendPacket(packet);
    }
    public void addInTelemetry(){
        telemetry.addLine(
                "============================== "
                        + name +  " =============================");

        for(int i = 0; i<Captions.size(); i++){
            telemetry.addData(Captions.get(i), Captions.get(i));
        }
        telemetry.addLine("======================================================================");
    }

}