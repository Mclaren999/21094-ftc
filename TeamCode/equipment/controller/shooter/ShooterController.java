package org.firstinspires.ftc.teamcode.equipment.controller.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ShooterController {

    private boolean useKI;
    public static double adaptiveRPM = 0, adaptivePosition = 0.2;
    private double currentRPM, currentPosition;

    public static boolean shooterIsActivity = false;

    public static double Ks = 5, Kh = 0.004;

    private double distance;

    public static double highRPM = 1700;

    public static double mediumRPM = 1210;
    public static double lowRPM = 1100;

    private double sumVelocity, hoodComp, rpmComp;

    public static double MAX_Y;

    public static double positionLow = 0;
    public static double positionHigh = 0.75;
    public static double positionMedium = 0.37;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime kdLeftBumper = new ElapsedTime();
    private ElapsedTime kdDpadDown = new ElapsedTime();
    private boolean isCoordinate = false;


    public void updateShooter(double coordinateX1, double coordinateY1, double coordinateX2, double coordinateY2, boolean leftBumper, boolean x, boolean a, boolean y, boolean b, boolean dpadDown, double velocityX, double velocityY, double xLL, double yLL) {
        if (dpadDown && kdDpadDown.milliseconds() > 300) {
            isCoordinate = !isCoordinate;
            kdDpadDown.reset();
        }
        if (isCoordinate) {
            updateAdaptive(coordinateX1, coordinateY1, coordinateX2, coordinateY2, leftBumper, velocityX, velocityY);
        } else {
            updateLL(xLL,yLL,leftBumper);
        }
    }

    private void updateCompensationSpeed(double velocityX, double velocityY) {
        sumVelocity = velocityX + velocityY;

        hoodComp = sumVelocity * -Kh;
        rpmComp = sumVelocity * -Ks;


    }

    private void updateAdaptive(double x1Pin, double y1Pin, double x2Pin, double y2Pin, boolean leftBumper, double velocityX, double velocityY) {

        if (leftBumper && kdLeftBumper.milliseconds() > 300 && !shooterIsActivity) {

            shooterIsActivity = true;
            kdLeftBumper.reset();

        } else if (leftBumper && kdLeftBumper.milliseconds() > 300 && shooterIsActivity) {

            currentRPM = 0;
            shooterIsActivity = false;
            kdLeftBumper.reset();

        }

        if (shooterIsActivity) {

            this.distance = Math.hypot(x2Pin - x1Pin, y2Pin - y1Pin);

            if (this.distance > 100) {
                currentRPM = highRPM;
                currentPosition = 0.75;
            } else if (this.distance < 56) {
                currentRPM = lowRPM;
                currentPosition = positionLow;
            } else {
                currentRPM = mediumRPM;
                currentPosition = positionMedium;



            }
        }
    }

    public void updateManual(boolean x, boolean a, boolean y,boolean leftBumper, boolean b){
        if (x) {// medium
            currentPosition = positionMedium;
            currentRPM = mediumRPM;
            shooterIsActivity = true;
        }
        else if (a) {// low
            currentPosition =  positionLow;
            currentRPM = lowRPM;

            shooterIsActivity = true;
        }else if(y){
            currentPosition = positionHigh;
            currentRPM = highRPM;
            shooterIsActivity = true;

        }else if (leftBumper) {//stop
            currentRPM = 0;
            shooterIsActivity = false;
        } else if (b) {
            currentRPM = adaptiveRPM;
            currentPosition =  adaptivePosition;
            shooterIsActivity = true;
        }
    }

    public void updateLL(double xLL,double yLL,boolean leftBumper){

        this.distance = Math.sqrt(xLL * xLL + yLL * yLL);

        if (leftBumper && kdLeftBumper.milliseconds() > 300 && !shooterIsActivity) {

            shooterIsActivity = true;
            kdLeftBumper.reset();

        } else if (leftBumper && kdLeftBumper.milliseconds() > 300 && shooterIsActivity) {

            currentRPM = 0;
            shooterIsActivity = false;
            kdLeftBumper.reset();

        }

        if (shooterIsActivity) {
            if (distance <= 3.2) {

                currentRPM = lowRPM;
                currentPosition = 0.2;

                useKI = false;

            } else if (distance <= 4.3) {

                currentPosition = 0.37;
                currentRPM = mediumRPM;

                useKI = false;

            } else {

                currentPosition = 0.75;
                currentRPM = highRPM;

                useKI = true;

            }
        }
    }





    public boolean isShooterIsActivity(){
        return shooterIsActivity;
    }
    public double getCurrentRPM(){
        return currentRPM;
    }
    public double getCurrentPosition(){
        return currentPosition;
    }

    public boolean isCoordinate(){
        return isCoordinate;
    }

    public boolean isUseKI(){return useKI;}

    public double getDistance(){
        return distance;
    }

}
//by chpenya
