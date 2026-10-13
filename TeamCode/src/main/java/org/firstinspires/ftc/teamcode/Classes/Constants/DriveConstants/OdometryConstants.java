package org.firstinspires.ftc.teamcode.Classes.Constants.DriveConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConstants {
    public static final double WHEEL_DIAMETER = 2; // Diámetro de la rueda (en pulgadas)
    public static final double TICKS_PER_REV = 8192; //8192; // Ticks por revolución del motor (ejemplo: Rev HD Hex)
    public static final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Circunferencia de la rueda

    public static  double FORWARD_OFFSET =  8.378893164651513;//3.0362142930768012; // Distancia entre ruedas izquierda y derecha (en pulgadas)
    public static  double LATERAL_OFFSET =   -1.2422859947974447;//-2.4833733135737694; // Distancia de la rueda lateral al centro del robot (en pulgadas)

    public static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
//48.39, -41.07, 42.98

    //BLUE
    //Theta:: 36.24348358532388
    //X:: -44.939815491203255
    //Y:: 44.71526378721703
    public static double Y_OffSet =  41.11481914370079; //10.5 BLUE
    public static double X_OffSet =  -41.57491849163386; //-10.5 Blue
    public static double Angular_Offset = 0;


}
