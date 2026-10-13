package org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug;

import org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.CameraConstants;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;
import org.opencv.core.Point;

public class CamMaths {
    public static double estimateXOffsetFromCenter(Point center, int imageWidth, double distanciaZ_cm) {
        double centerX = imageWidth / 2.0;
        double deltaPixels = center.x - centerX;

        double anglePerPixel = CameraConstants.FOV / imageWidth;
        double angleOffsetDeg = deltaPixels * anglePerPixel;
        double angleOffsetRad = Math.toRadians(angleOffsetDeg);

        return Math.tan(angleOffsetRad) * distanciaZ_cm;
    }


    public static Pose2D estimateObjectPose2(Pose2D robotPose, double distance, double distanceX){

        double robotAngleRadians = Math.toRadians(robotPose.theta+ OdometryConstants.Angular_Offset);
        double distanceY = Math.sqrt((distance*distance) - (distanceX*distanceX));
        double degreesCamAngle = Math.toDegrees(Math.atan2(distanceY, distanceX));
        double camAngle = Math.toRadians(degreesCamAngle-90);

        double angleRadians =robotAngleRadians + camAngle;
        double y = robotPose.y + (distance*Math.sin(angleRadians));
        double x = robotPose.x + (distance*Math.cos(angleRadians));

        return new Pose2D(y,x,0);

    }
}
