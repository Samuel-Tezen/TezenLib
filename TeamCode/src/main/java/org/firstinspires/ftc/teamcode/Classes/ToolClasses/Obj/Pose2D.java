package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class Pose2D {
    public final double y;
    public final double x;
    public double theta;
    public double time;

    public  Pose2D (double y, double x, double theta){
        this.y = y;
        this.x = x;
        this.theta = theta;
    }
    public  Pose2D (double y, double x, double theta, double time){
        this.y = y;
        this.x = x;
        this.theta = theta;
        this.time = time;
    }
    public static Pose2D fromFtcPose(AprilTagPoseFtc pose){
        return new Pose2D(pose.y, pose.x, pose.yaw);
    }
    public String toString(){
        return "y: " + y + " x: "+ x + " theta: " + theta;
    }
    public Pose2D toInch(){
        return new Pose2D(y/2.54,x/2.54, theta);
    }
    public Pose2D sum(Pose2D poseSum){
        return new Pose2D(this.y + poseSum.y,
                this.x + poseSum.x,
                this.theta + poseSum.theta);
    }
}
