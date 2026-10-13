package org.firstinspires.ftc.teamcode.Classes.ToolClasses.MotorControllers;


public class MotionProfileGenerator {

    private  double maxVelocity;
    private  double maxAcceleration;
    private  double distance;

    private  double tAccel;
    private  double tCruise;
    private  double tDecel;
    private  double totalTime;

    private boolean usingEdgesDistances = false;
    private MotionProfileGenerator profileY;
    private MotionProfileGenerator profileX;


    public MotionProfileGenerator(double distance, double maxVelocity, double maxAcceleration) {
        //Calculo de la escala de velocidad en toda una trayectoria basandose en la distancia y velocidad de l robot
        if (distance <= 0) {
            tAccel = 0;
            tDecel = 0;
            tCruise = 0;
            totalTime = 0;
            return;
        }
        this.distance = distance;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        tAccel = maxVelocity / maxAcceleration;
        double dAccel = 0.5 * maxAcceleration * Math.pow(tAccel, 2);
        if (2 * dAccel >= distance) {
            // Triangular profile
            tAccel = Math.sqrt(distance / maxAcceleration);
            tCruise = 0;
            tDecel = tAccel;
            totalTime = tAccel + tDecel;
        } else {
            // Trapezoidal profile
            double dCruise = distance - 2 * dAccel;
            tCruise = dCruise / maxVelocity;
            tDecel = tAccel;
            totalTime = tAccel + tCruise + tDecel;
        }
    }

    public double getVelocityAt(double t) {
        if (t < tAccel) {
            return maxAcceleration * t;
        } else if (t < tAccel + tCruise) {
            return maxVelocity;
        } else if (t < totalTime) {
            double tRel = t - (tAccel + tCruise);
            return maxVelocity - maxAcceleration * tRel;
        } else {
            return 0;
        }
    }
    public double getTotalTime() {
        return totalTime;
    }

    public double getAccelerationAt(double t) {
        if (t < tAccel) return maxAcceleration;
        else if (t < tAccel + tCruise) return 0;
        else if (t < totalTime) return -maxAcceleration;
        else return 0;
    }

    public void generateEdgesProfile(double[] distances){
        usingEdgesDistances = true;
        profileY = new MotionProfileGenerator(distances[1], maxVelocity, maxAcceleration);
        profileX= new MotionProfileGenerator(distances[0], maxVelocity, maxAcceleration);
    }
    public double[] getEdgeAccelAt(double t){
        if(!usingEdgesDistances){return null;}

       double xAccel = profileX.getAccelerationAt(t);
        double yAccel = profileY.getAccelerationAt(t);

        return new double[]{xAccel, yAccel};
    }

}
