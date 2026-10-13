package org.firstinspires.ftc.teamcode.Classes.RobotClasses.UpperBots.ChucoLib;


import com.acmerobotics.dashboard.config.Config;

/**
 * Calcula el ángulo y las RPM de la flywheel
 * según la distancia horizontal al goal.
 *
 * Los valores deben calibrarse experimentalmente.
 */
@Config
public final class ShooterCalculator {

    private ShooterCalculator() {
        // Clase de utilidades.
    }

    // =====================================================
    // Límites mecánicos del shooter
    // =====================================================

    public static double MIN_ANGLE_DEGREES = 22.0;
    public static double MAX_ANGLE_DEGREES = 55.0;

    // Distancia aceptada por el sistema.

    // =====================================================
    // Tabla de calibración
    // =====================================================
    //
    // Las RPM son los valores que ya estabas utilizando.
    //
    // Los ángulos son provisionales. Debes sustituirlos
    // con los valores reales medidos en cancha.
    // =====================================================

    public static double DISTANCE_1 = 34;
    public static double RPM_1 = 2000;
    public static double ANGLE_1 = 22;
    public static double BOOST_1 = 0.01;

    public static double DISTANCE_2 = 50.6;
    public static double RPM_2 = 2250;
    public static double ANGLE_2 = 30  ;
    public static double BOOST_2 = 0.024;


    public static double DISTANCE_3 = 85;
    public static double RPM_3 = 2500;
    public static double ANGLE_3 = 44;
    public static double BOOST_3 = 0.03;


    public static double DISTANCE_4 = 130.0;
    public static double RPM_4 = 3300;
    public static double ANGLE_4 = 46;
    public static double BOOST_4 = 0.05;



    public static boolean USE_ANGLE_TABLE = true;

    public static double MANUAL_ANGLE_DEGREES = 30.0;

    /**
     * Calcula el disparo completo.
     */
    public static ShotSolution calculate(double distanceInches) {


        double targetRPM = calculateRPM(distanceInches);
        double targetAngle = calculateAngle(distanceInches);
        double targetBoost = calculateBoost(distanceInches);

        return new ShotSolution(
                distanceInches,
                targetRPM,
                targetAngle,
                targetBoost
        );
    }

    /**
     * Calcula únicamente las RPM de la flywheel.
     */
    public static double calculateRPM(double distanceInches) {

        validateCalibrationDistances();

        return interpolateFourPoints(
                distanceInches,
                DISTANCE_1, RPM_1,
                DISTANCE_2, RPM_2,
                DISTANCE_3, RPM_3,
                DISTANCE_4, RPM_4
        );
    }

    /**
     * Calcula únicamente el ángulo.
     */
    public static double calculateBoost(double distanceInches){
        validateCalibrationDistances();
        double boost = interpolateFourPoints(
                distanceInches,
                DISTANCE_1, BOOST_1,
                DISTANCE_2, BOOST_2,
                DISTANCE_3, BOOST_3,
                DISTANCE_4, BOOST_4
        );
        return boost;
    }
    public static double calculateAngle(double distanceInches) {

        if (!USE_ANGLE_TABLE) {
            return clamp(
                    MANUAL_ANGLE_DEGREES,
                    MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES
            );
        }

        validateCalibrationDistances();

        double angle = interpolateFourPoints(
                distanceInches,
                DISTANCE_1, ANGLE_1,
                DISTANCE_2, ANGLE_2,
                DISTANCE_3, ANGLE_3,
                DISTANCE_4, ANGLE_4
        );

        return angle;
    }

    private static double interpolateFourPoints(
            double x,
            double x1,
            double y1,
            double x2,
            double y2,
            double x3,
            double y3,
            double x4,
            double y4
    ) {
        /*
         * No extrapolamos fuera de la tabla.
         * Utilizamos el primer o último valor.
         */
        if (x <= x1) {
            return y1;
        }

        if (x <= x2) {
            return linearInterpolation(
                    x,
                    x1, y1,
                    x2, y2
            );
        }

        if (x <= x3) {
            return linearInterpolation(
                    x,
                    x2, y2,
                    x3, y3
            );
        }

        if (x <= x4) {
            return linearInterpolation(
                    x,
                    x3, y3,
                    x4, y4
            );
        }

        return y4;
    }

    private static double linearInterpolation(
            double x,
            double x1,
            double y1,
            double x2,
            double y2
    ) {
        double difference = x2 - x1;

        if (Math.abs(difference) < 0.000001) {
            return y1;
        }

        double percentage =
                (x - x1) / difference;

        return y1
                + percentage
                * (y2 - y1);
    }

    private static void validateCalibrationDistances() {

        boolean validOrder =
                DISTANCE_1 < DISTANCE_2
                        && DISTANCE_2 < DISTANCE_3
                        && DISTANCE_3 < DISTANCE_4;

        if (!validOrder) {
            throw new IllegalStateException(
                    "Las distancias de calibración deben estar "
                            + "ordenadas de menor a mayor."
            );
        }
    }

    private static double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}