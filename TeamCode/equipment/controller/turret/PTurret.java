package org.firstinspires.ftc.teamcode.equipment.controller.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class PTurret {
    private DcMotorEx turret;
    private VoltageSensor voltageSensor;

    private double error;

    private double directionPos = 0.7;

    public static double kS = 0;    // Volts - static friction

    public static double kP = 0.008;
    public static double kI = 0;
    public static double kD = 0.0002;
    public static double kF = 0;


    private double targetPosition = 0;
    private double setPoint;
    private double measuredValue;
    private double minIntegral = -100, maxIntegral = 100;

    private double errorVal_p;
    private double errorVal_v;

    private double totalError;
    private double prevErrorVal;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    private static final double NOMINAL_VOLTAGE = 12.5;


    public void initialize(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");


        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


//            shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTargetPosition(double targetPosition) {
        setPoint = targetPosition;
    }

    public double getErrorAbsolute(){
        return Math.abs(targetPosition - turret.getCurrentPosition());
    }

    public void setDirectionPos(double setPos) {
        directionPos = setPos;
    }


    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }

    public void setSetPoint(double sp) {
        setPoint = sp;
        errorVal_p = setPoint - measuredValue;
        errorVal_v = (errorVal_p - prevErrorVal) / period;
    }

    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void update(double tolerance) {

        setTolerance(tolerance);

        prevErrorVal = errorVal_p;

        double pv = turret.getCurrentPosition();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (measuredValue == pv) {
            errorVal_p = setPoint - measuredValue;
        } else {
            errorVal_p = setPoint - pv;
            measuredValue = pv;
        }

        if (Math.abs(period) > 1E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        turret.setPower(kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint);
    }

    public boolean isReadyToPosition(double tolerance) {
        double currentPosition = turret.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) < tolerance;
    }
    public double getCurrentPosition(){
        return turret.getCurrentPosition();
    }

    public void showTelemetry(Telemetry telemetry) {
        double currentPosition = turret.getCurrentPosition();
        double currentVoltage = voltageSensor.getVoltage();
        double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

        telemetry.addLine("===  PIDSVA TURRET CONTROLLER ===");
        telemetry.addData("turret position: ", turret.getCurrentPosition());
        telemetry.addData("Target Pos", "%.1f", setPoint);
        telemetry.addData("Current Pos", "%.1f", currentPosition);
        telemetry.addData("Error RPM", "%.1f", setPoint - turret.getCurrentPosition());
        telemetry.addData("Motor Power", "%.3f", turret.getPower());
    }

}




