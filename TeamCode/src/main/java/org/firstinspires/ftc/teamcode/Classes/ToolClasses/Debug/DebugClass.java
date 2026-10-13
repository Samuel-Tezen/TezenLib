package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.ArrayList;

public class DebugClass {
    private final Telemetry telemetry;
    private final ArrayList<Zone> zones = new ArrayList<>();

    public DebugClass(Telemetry telemetry){
        this.telemetry = telemetry;
        addZone("Robot Pose");
    }

    public void addData(String caption, double data, int zone){
        zones.get(zone).addData(caption, data);
    }
    public void addZone(String name){
        zones.add(new Zone(telemetry, name));
    }

    public void update(Pose2D pose){
        for(int i = 0; i < zones.size(); i++){
            zones.get(i).addInTelemetry();
        }
        getZone("Robot Pose").drawRobot(pose);
        telemetry.update();
    }

    public Zone getZone(String name){
        for(int i = 0; i<zones.size(); i++){
            if(zones.get(i).name.equals(name)){
                return  zones.get(i);

            }
        }

        return null;
    }

    static class Double{
        public double num;
        public Double(double num){
            this.num = num;
        }

    }

}