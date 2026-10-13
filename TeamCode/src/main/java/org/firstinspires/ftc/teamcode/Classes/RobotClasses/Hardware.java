package org.firstinspires.ftc.teamcode.Classes.RobotClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
    public static DcMotorEx FR;
    public static DcMotorEx FL;
    public static DcMotorEx BR;
    public static DcMotorEx BL;


    public static IMU imu;

    public static ElapsedTime timer;

    public static boolean inited = false;
    public static boolean ChasisInited = false;
    public static boolean MotorToolsInited = false;
    public static boolean ServosInited = false;
    public static boolean ToolsInited = false;
    public static FtcDashboard ftcDashboard;


    public static GoBildaPinpointDriver pinpointDriver;


    public static void initAll(LinearOpMode linear){
        initChasis(linear);
        initIMU(linear);
        initFTCDashboard(linear);
        inited = true;
    }

    public static void initChasis( LinearOpMode linear){
        timer = new ElapsedTime();

        FR = linear.hardwareMap.get(DcMotorEx.class, "FR");
        FL = linear.hardwareMap.get(DcMotorEx.class, "FL");
        BR = linear.hardwareMap.get(DcMotorEx.class, "BR");
        BL = linear.hardwareMap.get(DcMotorEx.class, "BL");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);  // Modificar////
        FR.setDirection(DcMotorSimple.Direction.REVERSE);  // si
        BL.setDirection(DcMotorSimple.Direction.REVERSE);  // es/////
        BR.setDirection(DcMotorSimple.Direction.FORWARD );

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        pinpointDriver = linear.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//
//        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpointDriver.setOffsets(-168, 84, DistanceUnit.MM);
        ChasisInited = true;
    }
    /**
     * Inicializa el giroscopio
     **/
    public static void initIMU( LinearOpMode linear){
        IMU.Parameters myIMUparameters;
        imu = linear.hardwareMap.get(IMU.class, "imu");

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );


        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }

    public static void initFTCDashboard(LinearOpMode linear){
        ftcDashboard = FtcDashboard.getInstance();

    }

}
