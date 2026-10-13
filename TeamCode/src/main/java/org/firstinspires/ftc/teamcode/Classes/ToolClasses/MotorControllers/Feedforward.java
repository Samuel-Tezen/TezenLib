package org.firstinspires.ftc.teamcode.Classes.ToolClasses.MotorControllers;

// Simple feedforward for translational/rotational axes or individual wheels.
// Units: pick ONE and be consistent (e.g., m/s and m/s^2, or ticks/s and ticks/s^2).
public class Feedforward {
    private final double kS; // static friction term (output)
    private final double kV; // velocity gain (output per unit velocity)
    private final double kA; // acceleration gain (output per unit acceleration)
    private final boolean useVoltageComp;

    public Feedforward(double kS, double kV, double kA, boolean useVoltageComp) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.useVoltageComp = useVoltageComp;
    }

    // batteryVoltage: if useVoltageComp = true, pass measured battery voltage each loop.
    // signConvention: sign of desired velocity (keeps kS directionally correct).

    public double calculate(double velRef, double accRef, double batteryVoltage, double signConvention) {
        // Base feedforward (no voltage comp): u = kS*sgn + kV*v + kA*a
        double u = Math.signum(signConvention) * kS + kV * velRef + kA * accRef;

        if (useVoltageComp) {
            // If you send "power" to DcMotor (range [-1..1]), normalize by voltage
            // so u is relative to nominal 12V output.
            double v = Math.max(10.0, Math.min(14.0, batteryVoltage)); // guard
            u *= (12.0 / v);
        }
        return u;
    }
}
