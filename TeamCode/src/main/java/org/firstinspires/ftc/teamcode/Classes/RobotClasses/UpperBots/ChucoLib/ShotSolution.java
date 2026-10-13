package org.firstinspires.ftc.teamcode.Classes.RobotClasses.UpperBots.ChucoLib;

/**
 * Resultado calculado para un disparo.
 *
 * Todas las RPM corresponden a la flywheel, no al motor.
 */
public final class ShotSolution {

    private final double distanceInches;
    private final double flywheelRPM;
    private final double angleDegrees;
    private double boostPower;

    public ShotSolution(
            double distanceInches,
            double flywheelRPM,
            double angleDegrees
    ) {
        this.distanceInches = distanceInches;
        this.flywheelRPM = flywheelRPM;
        this.angleDegrees = angleDegrees;
    }
    public ShotSolution(
            double distanceInches,
            double flywheelRPM,
            double angleDegrees,
            double boostPower
    ) {
        this.distanceInches = distanceInches;
        this.flywheelRPM = flywheelRPM;
        this.angleDegrees = angleDegrees;
        this.boostPower = boostPower;
    }
    public double getBoostPower(){
        return boostPower;
    }

    public double getDistanceInches() {
        return distanceInches;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    @Override
    public String toString() {
        return String.format(
                "Distance: %.1f in | Flywheel: %.0f RPM | Angle: %.1f°",
                distanceInches,
                flywheelRPM,
                angleDegrees
        );
    }
}