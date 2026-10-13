package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.ColorAreaPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Cam Test", group="Vision")
public class CameraTest extends LinearOpMode {

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        // Obtener la webcam
        Hardware.initFTCDashboard(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Asignar pipeline
        webcam.setPipeline(new ColorAreaPipeLine());

        // Iniciar cámara
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
                FtcDashboard dashboard = FtcDashboard.getInstance();
                dashboard.startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error:", errorCode);
                telemetry.update();
            }
        });


        telemetry.addLine("Esperando start...");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Cámara corriendo...");
            telemetry.update();
            sleep(100);
        }
    }
}
