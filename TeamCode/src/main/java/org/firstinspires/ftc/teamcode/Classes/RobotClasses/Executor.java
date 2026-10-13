package org.firstinspires.ftc.teamcode.Classes.RobotClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Constants.MotionProfilingConstants;
import org.firstinspires.ftc.teamcode.Classes.Constants.PIDConstants.PIDConstants;
import org.firstinspires.ftc.teamcode.Classes.Constants.SVAConstants.ChasisSVA;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.UpperBots.UpperBot;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.Camera;
import org.firstinspires.ftc.teamcode.Classes.RobotClasses.vision.HuskyLensCamera;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Debug.DashboardUtil;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.MotorControllers.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.MotorControllers.PIDController;
import org.firstinspires.ftc.teamcode.Classes.ToolClasses.Obj.Pose2D;

import java.util.ArrayList;

public class Executor {
    private final Chasis chasis;
    ;
    private final ElapsedTime timer = new ElapsedTime();
    // PID Controllers
    private final PIDController yPID;
    private final PIDController xPID;
    private final PIDController headingPID;
    private final PIDController headingColorPID;
    private Camera camera;
    private final HuskyLensCamera huskyLens;
    private final UpperBot upperBot;

    private boolean inverted = false;
    private boolean yDirection = true;
    private boolean xDirection = true;
    private boolean angleDirection = false;



/*
    public Executor(Chasis chasis) {
        this.huskyLens = null;
        this.upperBot = null;
        this.chasis = chasis;
        this.chasis.linear.telemetry = Hardware.ftcDashboard.getTelemetry();
        // Configurar PIDControllers
        this.yPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.xPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.headingPID = new PIDController(PIDConstants.heading_KP, PIDConstants.heading_KI, PIDConstants.heading_KD);
        headingColorPID = null;


        yPID.setAreaIntegral(5);
        xPID.setAreaIntegral(5);
        headingPID.setAreaIntegral(10);
        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }
    public Executor(Chasis chasis, UpperBot upperBot) {
        this.huskyLens = null;
        this.upperBot = upperBot;
        this.chasis = chasis;
        this.chasis.linear.telemetry = Hardware.ftcDashboard.getTelemetry();
        // Configurar PIDControllers
        this.yPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.xPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.headingPID = new PIDController(PIDConstants.heading_KP, PIDConstants.heading_KI, PIDConstants.heading_KD);

        yPID.setAreaIntegral(5);
        xPID.setAreaIntegral(5);
        headingPID.setAreaIntegral(10);
        xPID.reset();
        yPID.reset();
        headingPID.reset();
        headingColorPID = null;
    }
    public Executor(Chasis chasis ,Camera camera) {
        this.huskyLens = null;
        this.upperBot = null;
        this.chasis = chasis;
        this.camera = camera;
        this.chasis.linear.telemetry = Hardware.ftcDashboard.getTelemetry();
        // Configurar PIDControllers
        this.yPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.xPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.headingPID = new PIDController(PIDConstants.heading_KP, PIDConstants.heading_KI, PIDConstants.heading_KD);

        yPID.setAreaIntegral(5);
        xPID.setAreaIntegral(5);
        headingPID.setAreaIntegral(10);
        xPID.reset();
        yPID.reset();
        headingPID.reset();
        headingColorPID = null;
    }
    public Executor(Chasis chasis, UpperBot upperBot, Camera camera) {
        this.huskyLens = null;
        this.upperBot = upperBot;
        this.chasis = chasis;
        this.camera = camera;
        this.chasis.linear.telemetry = Hardware.ftcDashboard.getTelemetry();
        // Configurar PIDControllers
        this.yPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.xPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.headingPID = new PIDController(PIDConstants.heading_KP, PIDConstants.heading_KI, PIDConstants.heading_KD);

        yPID.setAreaIntegral(5);
        xPID.setAreaIntegral(5);
        headingPID.setAreaIntegral(10);
        xPID.reset();
        yPID.reset();
        headingPID.reset();
        headingColorPID = null;
    }
    */

    public Executor(Chasis chasis, UpperBot upperBot, Camera camera, HuskyLensCamera huskyLens) {
        this.huskyLens = huskyLens;
        this.upperBot = upperBot;
        this.chasis = chasis;
        this.camera = camera;
        this.chasis.linear.telemetry = Hardware.ftcDashboard.getTelemetry();
        // Configurar PIDControllers
        this.yPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.xPID = new PIDController(PIDConstants.movement_KP, PIDConstants.movement_KI, PIDConstants.movement_KD);
        this.headingPID = new PIDController(PIDConstants.heading_KP, PIDConstants.heading_KI, PIDConstants.heading_KD);

        yPID.setAreaIntegral(5);
        xPID.setAreaIntegral(5);
        headingPID.setAreaIntegral(10);
        xPID.reset();
        yPID.reset();
        headingPID.reset();
        headingColorPID = null;
    }

    public static class Builder{
        Chasis chasis = null;
        UpperBot subsystem = null;
        Camera camera = null;
        HuskyLensCamera huskyLens = null;

        public Builder setChasis(Chasis chasis){
            this.chasis = chasis;
            return this;
        }
        public Builder setSubsystem(UpperBot upperBot){
            this.subsystem = upperBot;
            return this;
        }
        public Builder setCamera(Camera camera){
            this.camera = camera;
            return this;
        }
        public Builder setHuskyLens(HuskyLensCamera camera){
            this.huskyLens = camera;
            return this;
        }
        public Executor build(){
            return new Executor(chasis,subsystem,camera,huskyLens);
        }

    }

    public void followPath2(ArrayList<Pose2D> path) {

        final double followRadius = 5.0;
        final double finalTolerance = 0.2;
        final double headingTolerance = 3.0;

        if (path.size() < 2) return;

        double totalDistance = calculatePathLength(path);
        MotionProfileGenerator profile = new MotionProfileGenerator(totalDistance, MotionProfilingConstants.MaxVel, MotionProfilingConstants.MaxAccel);
        final double[] edgeDistances = getEdgesLenght(path);
        profile.generateEdgesProfile(edgeDistances);

        int index = 0;
        Pose2D lastPose = chasis.getPosition();

        yPID.reset();
        xPID.reset();
        headingPID.reset();
        timer.reset();

        while (chasis.linear.opModeIsActive()) {
            //Actualizar la posicion del chasis
            chasis.updatePosition();
            Pose2D current = chasis.getPosition();

            //Calcular el tiempo maximo que ocupa el robot para tarminar una trayectoria
            boolean passedTime = timer.seconds() >= profile.getTotalTime();
            if (index >= path.size() || passedTime) break;

            Pose2D target = path.get(index);


            double yError = target.y - current.y;
            double xError = target.x - current.x;
            double headingError = target.theta - current.theta;

            double distanceToPoint = Math.hypot(xError, yError);

            // Motion profiling
            double t = Math.min(timer.seconds(), profile.getTotalTime());
            double velocityTarget = profile.getVelocityAt(t);
            double scale = velocityTarget / MotionProfilingConstants.MaxVel;

            // Derivada para feedforward (velocidad deseada)
            double velX = 0;
            double velY = 0;
            if (index > 0 && index < path.size() - 1) {
                Pose2D prev = path.get(index - 1);
                Pose2D next = path.get(index + 1);
                double dt = next.time - prev.time;
                if (dt > 0) {
                    velX = (next.x - prev.x) / dt;
                    velY = (next.y - prev.y) / dt;
                }
            }
            double[] accel = profile.getEdgeAccelAt(t);

            // Feedforward
            double ffX = ChasisSVA.kV * velX +ChasisSVA. kA * accel[1] + Math.signum(velX) * ChasisSVA.kS;
            double ffY = ChasisSVA. kV * velY + ChasisSVA.kA * accel[0] + Math.signum(velY) * ChasisSVA.kS;

            // PID
            double yOut = ffY + (yPID.getOutput(yError, yDirection)*scale);
            double xOut = ffX + (xPID.getOutput(xError, xDirection)*scale);
            double headingOut = headingPID.getOutput(headingError, angleDirection);

            //Mueve el chasis con los calculos de PID y motion profiling
            chasis.power(yOut, xOut, headingOut);

            if(this.upperBot != null){
                this.upperBot.execute();
            }

            //Proyecta el robot en tiempo real en FTC dashboard
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(current, packet);
            DashboardUtil.drawPath(path, packet);
            DashboardUtil.sendPacket(packet);

            if (distanceToPoint < followRadius && index < path.size() - 1) {
                index++;
            }

            if (index == path.size() - 1 && distanceToPoint < finalTolerance && Math.abs(headingError) < headingTolerance) {
                break;
            }
        }
        chasis.power(0, 0, 0);
    }
    public void pause(double seconds){
        timer.reset();
        while(timer.seconds() < seconds){
            if(upperBot != null) {
                upperBot.execute();
                chasis.updatePosition();

            }

        }

    }
    public void followPath(ArrayList<Pose2D> path) {
        final double followRadius = 4.0;
        final double finalTolerance = 0.5;
        final double headingTolerance = 1.0;

        if (path.size() < 2) return;

        double totalDistance = calculatePathLength(path);
        MotionProfileGenerator profile = new MotionProfileGenerator(totalDistance, MotionProfilingConstants.MaxVel, MotionProfilingConstants.MaxAccel);

        int index = 0;
        double distanceTravelled = 0;
        Pose2D lastPose = chasis.getPosition();

        yPID.reset();
        xPID.reset();
        headingPID.reset();
        timer.reset();

        while (chasis.linear.opModeIsActive()) {
            chasis.updatePosition();
            Pose2D current = chasis.getPosition();

            boolean passedtime = timer.seconds()>= profile.getTotalTime();
            if (index >= path.size()||passedtime) break;

            Pose2D target = path.get(index);

            double yError = target.y - current.y;
            double xError = target.x - current.x;
            double headingError = target.theta - current.theta;

            double distanceToPoint = Math.hypot(xError, yError);

            // Calcular cuánto avanzamos desde el último ciclo
            double dx = current.x - lastPose.x;
            double dy = current.y - lastPose.y;
            double deltaDist = Math.hypot(dx, dy);
            distanceTravelled += deltaDist;
            lastPose = current;

            // Motion profiling
            double t = Math.min(timer.seconds(), profile.getTotalTime());
            double velocityTarget = profile.getVelocityAt(timer.seconds());
            double scale = velocityTarget / MotionProfilingConstants.MaxVel;

            // PID
            double yOut = yPID.getOutput(yError, yDirection) * scale;
            double xOut = xPID.getOutput(xError, xDirection) * scale;
            double headingOut = headingPID.getOutput(headingError, angleDirection);

            chasis.power(yOut, xOut, headingOut);
            if(this.upperBot != null){
                this.upperBot.execute();
            }

            if (distanceToPoint < followRadius && index < path.size() - 1) {
                index++;
            }

            if (index == path.size() - 1 && distanceToPoint < finalTolerance && Math.abs(headingError) < headingTolerance) {
                break;
            }


            // Debug
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(current, packet);
            DashboardUtil.drawPath(path, packet);
            packet.put("t", t);
            packet.put("targetVelocity", velocityTarget);
            DashboardUtil.sendPacket(packet);
            chasis.linear.telemetry.addData("Scale", scale);

            chasis.linear.telemetry.addData("Index", index);
            chasis.linear.telemetry.addData("Dist To Target", distanceToPoint);
            chasis.linear.telemetry.addData("Velocity Target", velocityTarget);
            chasis.linear.telemetry.addData("Scale", scale);
            chasis.linear.telemetry.addData("Distance Travelled", distanceTravelled);
            chasis.linear.telemetry.addData("Total distance", totalDistance);
            chasis.linear.telemetry.addData("Total time", profile.getTotalTime());


            chasis.linear.telemetry.addData("Time", timer.seconds());
            chasis.linear.telemetry.addData("Profile Duration", profile.getTotalTime());
            chasis.linear.telemetry.addData("Target Velocity", profile.getVelocityAt(timer.seconds()));
            chasis.linear.telemetry.update();
        }
        chasis.power(0, 0, 0);
    }

    public void invert(boolean red){
        this.inverted = red;
    }

    public void goToColor(double power,double time, String color){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        Pose2D current;

        while (timer.seconds() < time){
            double headingOutput = 0;
            chasis.updatePosition();
            current = chasis.getPosition();

            if(huskyLens != null) {
                if (huskyLens.getBloks() != null) {
                    chasis.setMaxAngularSpeed(0.2);
                    double x = 999999;
                    switch (color) {
                        case "Green":
                            x = huskyLens.getGreenX();
                            break;
                        case "Purple":
                            x = huskyLens.getPurpleX();
                            break;
                    }

                    if (x != 999999) {
                        headingOutput = headingColorPID.getOutput(150 - x, false);
                    }
                }
            }
            chasis.rawPower(power,0, headingOutput);
        }

    }

    public void goTo(Pose2D initial, Pose2D target){
        ArrayList<Pose2D> path = new ArrayList<>();
        path.add(initial);
        path.add(target);

        double totalDistance = calculatePathLength(path);
        timer.reset();
        MotionProfileGenerator profile = new MotionProfileGenerator(totalDistance, MotionProfilingConstants.MaxVel, MotionProfilingConstants.MaxAccel);

        boolean isFirstCycle = true;
        while (chasis.linear.opModeIsActive()) {
            chasis.updatePosition();
            Pose2D current = chasis.getPosition();

            double yError = target.y - current.y;
            double xError = target.x - current.x;
            double headingError = target.theta - current.theta;

            boolean passedTime = timer.seconds() >= profile.getTotalTime();
            if (passedTime){
                chasis.linear.telemetry.addLine("PASSED TIME");
                chasis.linear.telemetry.addData("Error X: ", xError);
                chasis.linear.telemetry.addData("Error Y: ", yError);

                chasis.linear.telemetry.update();

                break;

            }



            if(Math.abs(yError) < 0.2 && Math.abs(xError) < 0.2 && Math.abs(headingError)< 5){


                chasis.linear.telemetry.addLine("PASSED BY POSITION");
                chasis.linear.telemetry.addData(" Y error", yError);
                chasis.linear.telemetry.addData(" X error", xError);
                chasis.linear.telemetry.addData(" Theta error", headingError);
                chasis.linear.telemetry.update();
                break;
            }


            // Avance de distancia desde el último ciclo

            // Motion profiling
            double t = timer.seconds();
            double velocityTarget = profile.getVelocityAt(t);
            double scale = velocityTarget / MotionProfilingConstants.MaxVel;


            // PID corregido
            double yOut =   (yPID.getOutput(yError, yDirection)*scale);
            double xOut =   (xPID.getOutput(xError, xDirection)*scale);
            double headingOut = headingPID.getOutput(headingError, angleDirection);

            chasis.power(yOut, xOut, headingOut);
            if(upperBot != null){
                upperBot.execute();
            }


//            if(camera != null){
//                Pose2D velocity2d = chasis.getVelocity();
//                double velocity = Math.hypot(Math.abs(velocity2d.x), Math.abs(velocity2d.y));
//                double angularVelocity = velocity2d.theta;
//
//                if(camera.isAprilTagDetected() && velocity <= 14 && Math.abs(angularVelocity) <= 7) {
//                    Pose2D campose = camera.getRobotPoseFromAprilTag();
//                    if(campose != null){
//                        chasis.setPosition(campose);
//                    }
//                }
//            }
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(current, packet);
            DashboardUtil.drawPath(path, packet);
            DashboardUtil.sendPacket(packet);

        }
        chasis.power(0,0,0);

    }


    private double calculatePathLength(ArrayList<Pose2D> path) {
        double total = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2D p1 = path.get(i);
            Pose2D p2 = path.get(i + 1);
            total += Math.hypot(p2.x - p1.x, p2.y - p1.y);
        }
        return total;
    }


    //CAMERA LOGITECH
    public void alignToColor(String color, double objectHeight, Pose2D targetFromPose){
        if(this.camera == null){
            return;
        }
        chasis.updatePosition();
        Pose2D currentPose = chasis.getPosition();
        Pose2D objectPose = camera.getColorPose(color, objectHeight, currentPose);

        Pose2D robotTarget = objectPose.sum(targetFromPose);

        if(robotTarget != null) {
            goTo(currentPose, robotTarget);

        }


    }

    public double[] getEdgesLenght(ArrayList<Pose2D> path){
        double totalX = 0;
        double totalY = 0;

        for(int i = 0; i < path.size()-1; i++){
            totalX += Math.abs(path.get(i).x);
            totalY += Math.abs(path.get(i).y);

        }
        return new double[]{totalX, totalY};
    }

    //HUSKYLENS GOBILDA
    public void goToAndAligntoColor(Pose2D initial, Pose2D target, String color){
        ArrayList<Pose2D> path = new ArrayList<>();
        path.add(initial);
        path.add(target);
        double totalDistance = calculatePathLength(path);
        timer.reset();
        MotionProfileGenerator profile = new MotionProfileGenerator(totalDistance, MotionProfilingConstants.MaxVel, MotionProfilingConstants.MaxAccel);
        while (chasis.linear.opModeIsActive()) {
            chasis.updatePosition();
            huskyLens.update();
            Pose2D current = chasis.getPosition();

            boolean passedTime = timer.seconds() >= profile.getTotalTime();

            if (passedTime) break;

            double yError = target.y - current.y;
            double xError = target.x - current.x;
            double headingError = target.theta - current.theta;

            if(Math.abs(yError) < 0.2 && Math.abs(xError) < 0.2){


                chasis.linear.telemetry.addLine("PASSED TIME");
                chasis.linear.telemetry.addData(" Y error", yError);
                chasis.linear.telemetry.addData(" X error", xError);
                chasis.linear.telemetry.addData(" Theta error", headingError);
                chasis.linear.telemetry.update();
                break;
            }
            // Avance de distancia desde el último ciclo

            // Motion profiling
            double t = timer.seconds();
            double velocityTarget = profile.getVelocityAt(t);
            double scale = velocityTarget / MotionProfilingConstants.MaxVel;


            // PID corregido
            double yOut =   (yPID.getOutput(yError, yDirection)*scale);
            double xOut =   (xPID.getOutput(xError, xDirection)*scale);
            double headingPIDOutput = headingPID.getOutput(headingError, angleDirection);


            if(huskyLens.getBloks() != null){
                chasis.setMaxAngularSpeed(0.2);
                double x = 999999;
                switch (color){
                    case "Green":
                        x = huskyLens.getGreenX();
                        break;
                    case "Purple":
                        x = huskyLens.getPurpleX();
                        break;
                }

                if(x != 999999) {
                    headingPIDOutput = headingColorPID.getOutput(150 - x, false);
                }
            }


            chasis.power(yOut, xOut, headingPIDOutput);
            if(upperBot != null){
                upperBot.execute();
            }
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(current, packet);
            DashboardUtil.drawPath(path, packet);
            DashboardUtil.sendPacket(packet);

        }
        chasis.setMaxAngularSpeed(1);
        chasis.power(0,0,0);

    }

    public void goToAndAligntoAprilTag(Pose2D initial, Pose2D target, String color){
        ArrayList<Pose2D> path = new ArrayList<>();
        path.add(initial);
        path.add(target);
        double totalDistance = calculatePathLength(path);
        MotionProfileGenerator profile = new MotionProfileGenerator(totalDistance, MotionProfilingConstants.MaxVel, MotionProfilingConstants.MaxAccel);
        timer.reset();
        boolean detected = false;
        while (chasis.linear.opModeIsActive()) {
            chasis.updatePosition();
            huskyLens.update();
            Pose2D current = chasis.getPosition();

            boolean passedTime = timer.seconds() >= profile.getTotalTime();

            if (passedTime) break;

            double yError = target.y - current.y;
            double xError = target.x - current.x;
            double headingError = target.theta - current.theta;

            if(yError < 0.2 && xError < 0.2 && headingError < 1) break;

            // Avance de distancia desde el último ciclo

            // Motion profiling
            double t = timer.seconds();
            double velocityTarget = profile.getVelocityAt(t);
            double scale = velocityTarget / MotionProfilingConstants.MaxVel;


            // PID corregido
            double yOut =   (yPID.getOutput(yError, yDirection)*scale);
            double xOut =   (xPID.getOutput(xError, xDirection)*scale);
            double headingPIDOutput = headingPID.getOutput(headingError, false);


            if(huskyLens.getBloks() != null){
                detected = true;
                chasis.setMaxAngularSpeed(0.4);
                double x = 999999;
                switch (color){
                    case "Green":
                        x = huskyLens.getGreenX();
                        break;
                    case "Purple":
                        x = huskyLens.getPurpleX();
                        break;
                }

                if(x != 999999) {
                    headingPIDOutput = headingColorPID.getOutput(150 - x, angleDirection);
                }
            }


            chasis.power(yOut, xOut, headingPIDOutput);
            if(upperBot != null){
                upperBot.execute();
            }
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(current, packet);
            DashboardUtil.drawPath(path, packet);
            DashboardUtil.sendPacket(packet);

        }
        chasis.power(0,0,0);

    }


}

