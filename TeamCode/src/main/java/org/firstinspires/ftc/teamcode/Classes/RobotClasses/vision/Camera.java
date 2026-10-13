package org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision;

import static org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.CameraConstants.CameraHeight;
import static org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.CameraConstants.FocalLength;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.TensorFlow.MLVisionProcessor;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.CamMaths;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

import java.util.List;

public class Camera {
    // ML processor fields

    // Config ML (ajusta a tu modelo)
    private static final String ML_MODEL_PATH = "tf/model.tflite";
    private static final String ML_LABELS_PATH = "tf/labels.txt";
    private static final int    ML_INPUT_SIZE = 300;      // ej. 300 o 320
    private static final boolean ML_QUANTIZED = true;    // true si tu modelo es uint8
    private static final int    ML_MAX_DETS   = 10;
    private static final float  ML_SCORE_TH   = 0.55f;

    private final MLVisionProcessor mlProcessor;

    VisionPortal visionPortal;
    ColorProcessor colorProcessor;
    AprilTagProcessor aprilTagProcessor;
    Position camPosition = new Position(DistanceUnit.INCH,0.5, 7, 12,0);

    Size size = new Size(1280, 800);
    public Camera (LinearOpMode linear){
        // Tu pipeline de OpenCV
        colorProcessor = new ColorProcessor();

        mlProcessor = new MLVisionProcessor(
                linear.hardwareMap.appContext,
                ML_MODEL_PATH,
                ML_LABELS_PATH,
                ML_INPUT_SIZE,
                ML_QUANTIZED,
                ML_MAX_DETS,
                ML_SCORE_TH
        );

        YawPitchRollAngles cameraOrientation =
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -60, 0, 0L);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(
                        915.0,  // fx
                        915.0,  // fy
                        640.0,  // cx
                        400.0   // cy
                ).setCameraPose(camPosition, cameraOrientation)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(linear.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(size)
                .addProcessor(aprilTagProcessor)
                .build();


    }

    public void start() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming(); // reinicia la cámara si se pausó
            FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.stopStreaming(); // apaga la cámara para liberar recursos
        }
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;

    }

    public AprilTagDetection getBestAprilTagDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        AprilTagDetection best = null;
        double bestScore = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
          //  if (detection.metadata == null) continue;
            if (detection.robotPose == null) continue;

            /*
             * Criterio simple:
             * preferimos tags más cercanos.
             * Después podemos mejorarlo con margen de decisión,
             * ángulo, cantidad de tags, etc.
             */
            double score = detection.ftcPose.range;

            if (score < bestScore) {
                bestScore = score;
                best = detection;
            }
        }

        return best;
    }


    public double getDistance(){
        AprilTagDetection detection = getBestAprilTagDetection();

        if (detection == null || detection.robotPose == null) {
            return 0;
        }
        return detection.ftcPose.range;

    }
    public double getDistanceX(){
        AprilTagDetection detection = getBestAprilTagDetection();

        if (detection == null || detection.robotPose == null) {
            return 0;
        }
        return detection.ftcPose.bearing;
    }

    public int getTagID(){
        AprilTagDetection detection = getBestAprilTagDetection();

        if (detection == null || detection.robotPose == null ) {
            return 0;
        }

        return 1;
    }
    public Pose2D getRobotPoseFromAprilTag() {
        AprilTagDetection detection = getBestAprilTagDetection();


        if (detection == null || detection.robotPose == null ) {
            return null;
        }
//        if(detection.id == 21 ||detection.id == 22||detection.id == 23){
//            return null;
//
//        }

        /*
         * detection.robotPose es la posición global del robot
         * en el sistema de coordenadas de la cancha.
         *
         * Dependiendo de tu clase Pose2D, ajusta el constructor.     
         */
//        double x = detection.ftcPose.x;
//        double y = detection.ftcPose.y;
          double x = detection.robotPose.getPosition().y;
          double y = -detection.robotPose.getPosition().x;
          double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)-90;

        return new Pose2D(y, x, heading);
    }
    public boolean isAprilTagDetected() {
        return !aprilTagProcessor.getDetections().isEmpty() ;
    }



//======================= Color processor ================================
    public ColorProcessor getColorProcessor() {
        return colorProcessor;
    }
    private ColorProcessor.Color toColor(String color){
        switch (color){
            case "red":
                return ColorProcessor.Color.RED;
            case "blue":
                return ColorProcessor.Color.BLUE;
            case "yellow":
                return ColorProcessor.Color.YELLOW;

        }
        return ColorProcessor.Color.RED;
    }

    public Pose2D getColorPose(String color, double height, Pose2D robotPose){
        return colorProcessor.getObjectFieldPose(color, height, robotPose);
    }

    public double getColorDistance(String color, double height){

        return colorProcessor.getColorDistance(toColor(color), height);
    }

    public double getColorStrafe(String color, double height){

        return colorProcessor.getColorStrafe(toColor(color), height);
    }

    public List<MLVisionProcessor.Detection2D> getMLDetections() {
        if (mlProcessor == null) return java.util.Collections.emptyList();
        return mlProcessor.getLatestDetections();
    }
    public void enableML(boolean enable) {
        if (visionPortal == null || mlProcessor == null) return;
        if (enable) {
            visionPortal.setProcessorEnabled(mlProcessor, true);
        } else {
            visionPortal.setProcessorEnabled(mlProcessor, false);
        }
    }
    public void close() {
        try {
            if (mlProcessor != null) mlProcessor.close();
        } catch (Exception ignored) {}
        // ... cierre de portal o demás recursos
    }



}
