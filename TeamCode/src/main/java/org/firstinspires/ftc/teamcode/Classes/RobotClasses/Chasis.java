package org.firstinspires.ftc.teamcode.Classes.RobotClasses;

import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.Angular_Offset;
import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.LATERAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.TICKS_TO_INCHES;
import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.X_OffSet;
import static org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants.OdometryConstants.Y_OffSet;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Encoder;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

public class Chasis{
    private final DcMotorEx FR, FL, BR, BL;
    private final IMU imu;
    private final Encoder Y_Encoder, X_Encoder;
    private double lastRawAngle = 0;
    public double accumulatedAngle = 0;
    private  double MaxSpeed = 1, MaxAngularSpeed = 1;
    private double lastForwardTicks = 0;
    private double lastLateralTicks = 0;
    private double lastHeading = 0;

    private double lastX = X_OffSet;
    private double lastY = Y_OffSet;
    private double lastAngle = 0;

    private double xPosition = X_OffSet;
    private double yPosition = Y_OffSet;
    private double angle = 0;

    private GoBildaPinpointDriver pinpointDriver;

    public LinearOpMode linear;

    private boolean UsingDriver = false;

    public void UseDriver(boolean usingDriver){
        this.UsingDriver = usingDriver;
        if(usingDriver){
            pinpointDriver = Hardware.pinpointDriver;
            pinpointDriver.recalibrateIMU();
            pinpointDriver.resetPosAndIMU();
        }

    }

    public Chasis(LinearOpMode linear){
        this.linear = linear;
        if(!Hardware.ChasisInited){
            Hardware.initChasis(linear);
            Hardware.initIMU(linear);

        }


        this.FR = Hardware.FR;
        this.FL = Hardware.FL;
        this.BR = Hardware.BR;
        this.BL = Hardware.BL;

        Y_Encoder = new Encoder(BR);
        Y_Encoder.setDirection(false);

        X_Encoder = new Encoder(FL);
        X_Encoder.setDirection(false);

        imu = Hardware.imu;


    }

    public void power(double y, double x, double omega)     {
        //Limitar la velocidad
        if(y > MaxSpeed){
            y = MaxSpeed;
        } else if (y < -MaxSpeed) {
            y = -MaxSpeed;
        }
        if(x > MaxSpeed){
            x = MaxSpeed;
        } else if (x < -MaxSpeed) {
            x = -MaxSpeed;
        }
        if(omega > MaxAngularSpeed){
            omega = MaxAngularSpeed;
        } else if (omega < -MaxAngularSpeed) {
            omega = -MaxAngularSpeed;
        }

        //Globalizacion de las velocidades para el FOD
        double angleRadians = Math.toRadians(-angle);
        double globalPowerX = x * Math.cos(angleRadians) - y * Math.sin(angleRadians);
        double globalPowerY = x * Math.sin(angleRadians) + y * Math.cos(angleRadians);

        double frontLeftPower = globalPowerY + globalPowerX + omega;
        double frontRightPower = globalPowerY - globalPowerX - omega;
        double backLeftPower = globalPowerY - globalPowerX + omega;
        double backRightPower = globalPowerY + globalPowerX - omega;


        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }
    public void rawPower(double y, double x, double omega){
        if(y > MaxSpeed){
            y = MaxSpeed;
        } else if (y < -MaxSpeed) {
            y = -MaxSpeed;
        }
        if(x > MaxSpeed){
            x = MaxSpeed;
        } else if (x < -MaxSpeed) {
            x = -MaxSpeed;
        }
        if(omega > MaxAngularSpeed){
            omega = MaxAngularSpeed;
        } else if (omega < -MaxAngularSpeed) {
            omega = -MaxAngularSpeed;
        }

        double frontLeftPower = y - x - omega;
        double frontRightPower = y + x + omega;
        double backLeftPower = y + x - omega;
        double backRightPower = y - x + omega;


        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }
    public void powerInAngle(double y, double x, double omega, double angleOffset){
        //Limitar la velocidad
        if(y > MaxSpeed){
            y = MaxSpeed;
        } else if (y < -MaxSpeed) {
            y = -MaxSpeed;
        }
        if(x > MaxSpeed){
            x = MaxSpeed;
        } else if (x < -MaxSpeed) {
            x = -MaxSpeed;
        }
        if(omega > MaxAngularSpeed){
            omega = MaxAngularSpeed;
        } else if (omega < -MaxAngularSpeed) {
            omega = -MaxAngularSpeed;
        }

        //Globalizacion de las velocidades para el FOD
        double angleRadians = Math.toRadians((angle+angleOffset) * 1);
        double globalPowerX = x * Math.cos(angleRadians) - y * Math.sin(angleRadians);
        double globalPowerY = x * Math.sin(angleRadians) + y * Math.cos(angleRadians);

        double frontLeftPower = globalPowerY - globalPowerX - omega;
        double frontRightPower = globalPowerY + globalPowerX + omega;
        double backLeftPower = globalPowerY + globalPowerX - omega;
        double backRightPower = globalPowerY - globalPowerX + omega;


        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }

    public void updatePosition() {
        if(!UsingDriver) {
            double currentHeading = getAccumulatedAngle();
            double deltaHeading = Math.toRadians(currentHeading - lastHeading); // Ahora en radianes

            double currentForwardTicks = Y_Encoder.getPosition();
            double currentLateralTicks = X_Encoder.getPosition();

            double deltaForwardTicks = currentForwardTicks - lastForwardTicks;
            double deltaLateralTicks = currentLateralTicks - lastLateralTicks;

            lastForwardTicks = currentForwardTicks;
            lastLateralTicks = currentLateralTicks;
            lastHeading = currentHeading;

            double deltaForward = deltaForwardTicks * TICKS_TO_INCHES;
            double deltaLateral = deltaLateralTicks * TICKS_TO_INCHES;

            //  Compensación de rotación

           // double globalPowerX = x * Math.cos(angleRadians) - y * Math.sin(angleRadians);
           // double globalPowerY = x * Math.sin(angleRadians) + y * Math.cos(angleRadians);

            double deltaForwardComp = deltaForward - (deltaHeading * FORWARD_OFFSET);
            double deltaLateralComp = deltaLateral - (deltaHeading * LATERAL_OFFSET);

            double headingRad = Math.toRadians(currentHeading);
            double cosHeading = Math.cos(headingRad);
            double sinHeading = Math.sin(headingRad);

            double deltaX = deltaLateralComp * cosHeading - deltaForwardComp * sinHeading;
            double deltaY = deltaLateralComp * sinHeading + deltaForwardComp * cosHeading;

            xPosition += deltaX;
            yPosition += deltaY;


            angle = currentHeading; // Actualizar heading correctamente

        }else{
            //Actualizar la posicion del chasis con las death wheels
            //pinpointDriver.update();

            //Sacar los valores del sensor
            double xRawPosition = pinpointDriver.getPosX(DistanceUnit.INCH) + X_OffSet;
            double yRawPosition = pinpointDriver.getPosY(DistanceUnit.INCH) + Y_OffSet;
            double RawAngle = pinpointDriver.getPosition().getHeading(AngleUnit.DEGREES) + Angular_Offset;

            //Saca la diferencia entre las coordenadas
            double deltaX = xRawPosition - lastX;
            double deltaY = yRawPosition - lastY;
            double deltaTheta = RawAngle - lastAngle;

            //La coordenada actual pasa a ser la anterior
            lastX = xRawPosition;
            lastY = yRawPosition;
            lastAngle = RawAngle;

            //Se suma la diferencia de coordenadas a la coordenada actual
            xPosition += deltaX;
            yPosition +=deltaY;
            angle += deltaTheta;


        }

    }

    private boolean angleInitialized = false;


    private double getAccumulatedAngle() {

        double rawAngle = getYaw();

        // Primera lectura: sincronizar con el IMU
        if (!angleInitialized) {
            lastRawAngle = rawAngle;
            angleInitialized = true;
            return accumulatedAngle;
        }

        double deltaAngle = rawAngle - lastRawAngle;

        // Corregir wrap-around del IMU
        if (deltaAngle > 140.0) {
            deltaAngle -= 360.0;
        } else if (deltaAngle < -140.0) {
            deltaAngle += 360.0;
        }

        accumulatedAngle += deltaAngle;

        lastRawAngle = rawAngle;

        return accumulatedAngle;
    }
    public void setPinpointPosition(double x, double y , double theta){
        pinpointDriver.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, theta));
    }
    public void resetPosition() {
        Y_Encoder.reset();
        X_Encoder.reset();

        imu.resetYaw();

        lastForwardTicks = 0;
        lastLateralTicks = 0;
        lastHeading = 0;

        lastRawAngle = 0;
        accumulatedAngle = 0;
        angleInitialized = false;

        angle = 0;

        xPosition = X_OffSet;
        yPosition = Y_OffSet;

        if (UsingDriver) {
            pinpointDriver.recalibrateIMU();
            pinpointDriver.resetPosAndIMU();

            lastY = Y_OffSet;
            lastX = X_OffSet;
            lastAngle = 0;
        }
    }
    private double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public Pose2D getVelocity(){
        return new Pose2D(pinpointDriver.getVelY(DistanceUnit.INCH),
                pinpointDriver.getVelX(DistanceUnit.INCH),
                pinpointDriver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
    }
    public Pose2D getPosition(){
        return new Pose2D(yPosition, xPosition, angle);
    }
    public void setPosition(Pose2D position){
        X_OffSet = 0;
        Y_OffSet = 0;
        Angular_Offset = 0;

        xPosition = position.x;
        yPosition = position.y;
        angle = position.theta;

        lastX = position.x;
        lastY = position.y;
        lastAngle = position.theta;

        if(UsingDriver){
            pinpointDriver.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH,
                    xPosition,yPosition, AngleUnit.DEGREES, angle));

        }

    }
    public void setMaxSpeed(double maxSpeed) {
        MaxSpeed = maxSpeed;
    }
    public void setMaxAngularSpeed(double maxAngularSpeed) {
        MaxAngularSpeed = maxAngularSpeed;
    }
    public double getForwardEncoderPosition() {
        return Y_Encoder.getPosition();
    }
    public double getLateralEncoderPosition() {
        return X_Encoder.getPosition();
    }

    public static double calculateAngleToTarget(Pose2D RobotPose, Pose2D targetPose){
        return Math.toDegrees(Math.atan2(targetPose.y - RobotPose.y,targetPose.x - RobotPose.x))-90;
        //return Math.toDegrees(Math.atan2(RobotPose.x + targetPose.x,RobotPose.y + targetPose.y));
    }
    public double calculateDistanceToTarget(Pose2D RobotPose, Pose2D targetPose){
        return Math.hypot(targetPose.y - RobotPose.y,targetPose.x - RobotPose.x);
    }

}
