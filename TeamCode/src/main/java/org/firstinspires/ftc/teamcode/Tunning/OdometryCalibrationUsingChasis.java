package org.firstinspires.ftc.teamcode.Tunning;

import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.TICKS_TO_INCHES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Chasis;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.Hardware;

@TeleOp(name = "Calibrar Offsets Odometría", group = "Calibration")
public class OdometryCalibrationUsingChasis extends LinearOpMode {

    private Chasis chasis;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initIMU(this);
        Hardware.initChasis(this);
        Hardware.initFTCDashboard(this);
        chasis = new Chasis(this);
        chasis.UseDriver(false); //If using pinpoint driver: true
        chasis.resetPosition();
        this.telemetry = Hardware.ftcDashboard.getTelemetry();

        telemetry.addLine("Listo para calibrar. Presiona START.");
        telemetry.update();

        waitForStart();

        // --- Medir antes de rotar ---
        double startHeading = chasis.getPosition().theta;
        double startForwardTicks = chasis.getForwardEncoderPosition();
        double startLateralTicks = chasis.getLateralEncoderPosition();

        // --- Rotar el robot en su lugar ---
        double rotationPower = -0.25;
        double targetDegrees = -360;
        chasis.setMaxSpeed(1);// Giro completo
        chasis.setMaxAngularSpeed(1);

        while (opModeIsActive()) {
            chasis.updatePosition();
            chasis.power(0, 0, rotationPower); // Girar sobre sí mismo


            telemetry.addData("Heading", chasis.getPosition().theta-startHeading);
            telemetry.update();

            if((chasis.getPosition().theta-startHeading)<targetDegrees){
                telemetry.addData("EYYY", chasis.getPosition().theta-startHeading);

                break;
            }
        }

        // Frenar motores
        chasis.power(0, 0, 0);

        // --- Medir después de rotar ---
        double endHeading = chasis.getPosition().theta;
        double endForwardTicks = chasis.getForwardEncoderPosition();
        double endLateralTicks = chasis.getLateralEncoderPosition();

        // --- Calcular diferencias ---
        double deltaHeading = endHeading - startHeading;
        double deltaForwardTicks = endForwardTicks - startForwardTicks;
        double deltaLateralTicks = endLateralTicks - startLateralTicks;

        // Conversión Ticks -> Pulgadas
        double deltaForwardInches = deltaForwardTicks * TICKS_TO_INCHES;
        double deltaLateralInches = deltaLateralTicks * TICKS_TO_INCHES;

        // --- Calcular Offsets ---
        double forwardOffset = deltaForwardInches / Math.toRadians(deltaHeading);
        double lateralOffset = deltaLateralInches / Math.toRadians(deltaHeading);

        telemetry.addLine("------ Resultados de Calibración ------");
        telemetry.addData("Forward Offset (Y encoder)", forwardOffset);
        telemetry.addData("Lateral Offset (X encoder)", lateralOffset);
        telemetry.update();


        sleep(10000); // Dejar tiempo para ver los resultados
    }
}

