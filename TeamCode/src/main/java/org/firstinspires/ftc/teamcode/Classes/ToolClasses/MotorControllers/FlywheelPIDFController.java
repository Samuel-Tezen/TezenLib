package org.firstinspires.ftc.teamcode.Classes.ToolClasses.MotorControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelPIDFController {

    private double kP;
    private double kI;
    private double kD;

    // Feedforward
    private double kV;
    private double kA;
    private double kS;

    private double targetRPM = 0.0;
    private double lastTargetRPM = 0.0;

    private double integralSum = 0.0;
    private double lastError = 0.0;

    private double minOutput = 0.0;
    private double maxOutput = 1.0;

    private double integralZoneRPM = 150.0;
    private double maxIntegralSum = 1000.0;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean desactivateNegativePID = false;

    public FlywheelPIDFController(
            double kP,
            double kI,
            double kD,
            double kV,
            double kA,
            double kS
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;

        timer.reset();
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double calculate(double currentRPM) {
        double dt = timer.seconds();
        timer.reset();

        if (dt < 0.005) {
            dt = 0.005;
        }

        if (targetRPM <= 0.0) {
            reset();
            lastTargetRPM = 0.0;
            return 0.0;
        }

        double error = targetRPM - currentRPM;

        // PID
        if (Math.abs(error) <= integralZoneRPM) {
            integralSum += error * dt;
            integralSum = clamp(integralSum, -maxIntegralSum, maxIntegralSum);
        } else {
            integralSum = 0.0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidOutput =
                (kP * error) +
                        (kI * integralSum) +
                        (kD * derivative);
        if (desactivateNegativePID) {
            pidOutput = Math.max(pidOutput, 0.0);
        }

        // Feedforward
        double targetAccelerationRPMPerSec = (targetRPM - lastTargetRPM) / dt;
        lastTargetRPM = targetRPM;

        double feedforwardOutput =
                (kS * Math.signum(targetRPM)) +
                        (kV * targetRPM) +
                        (kA * targetAccelerationRPMPerSec);

        double output = feedforwardOutput + pidOutput;

        return clamp(output, minOutput, maxOutput);
    }

    public double calculate(double targetRPM, double currentRPM) {
        setTargetRPM(targetRPM);
        return calculate(currentRPM);
    }

    public boolean atTarget(double currentRPM, double toleranceRPM) {
        return Math.abs(targetRPM - currentRPM) <= toleranceRPM;
    }
    public boolean atTargetWhitBoost(double currentRPM, double toleranceRPM){
        return  currentRPM > targetRPM && currentRPM < targetRPM+toleranceRPM;

    }
    public boolean greaterThanTarget(double currentRPM){
        return  currentRPM > targetRPM;
    }

    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setFeedforward(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public void setOutputLimits(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setIntegralZoneRPM(double integralZoneRPM) {
        this.integralZoneRPM = Math.abs(integralZoneRPM);
    }

    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = Math.abs(maxIntegralSum);
    }

    public double getVelocityFeedforward() {
        return kV * targetRPM;
    }

    public double getStaticFeedforward() {
        return kS * Math.signum(targetRPM);
    }

    public double getError(double currentRPM) {
        return targetRPM - currentRPM;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void desactivateNegativePID() {
        desactivateNegativePID = true;
    }
    public void activateNegativePID() {
        desactivateNegativePID = false;
    }
}