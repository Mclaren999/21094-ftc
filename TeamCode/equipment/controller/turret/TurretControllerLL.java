package org.firstinspires.ftc.teamcode.equipment.controller.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.equipment.controller.vision.MegaTag2Controller;
import org.firstinspires.ftc.teamcode.equipment.implementation.Limelight;


@Config
public class TurretControllerLL {

    private double DIVISIONPRICE = 2.688888888888889;

    private ElapsedTime kdButton = new ElapsedTime();
    public static double toleranceInvalidSecond = 800;

    private boolean isTurretStart = false,isUsingHeading = true;


    public static double mediumAngle = -42,lowAngle = -53,highAngle = -46, currentModeAngle, offsetAngle;

    private double currentAngleTx, targetTx,TurretAngle;

    private final double FULL_TURN = 1196;

    private final double MAX_TURRET = 848;
    private final double MIN_TURRET = -848;

    private double currentAngle;

    private boolean isEnable = true;
    private double offset;

    private double limelightAngle = 0;


    private boolean isLimelightTarget = false;
    private double currentComp,lastComp;
    private double normalizedTurret;
    private double sumAngle;
    private double targetPosition;
    private PTurret pTurret = new PTurret();

    private Limelight limelight = new Limelight();
    private double currentAnglePin, lastAnglePin;

    private ElapsedTime switchKdButton = new ElapsedTime();

    private double MAX_X = - 3.2;

    private double angleGoal = 0;

    private double Heading;

    private MegaTag2Controller.targetState targetState;

    private MegaTag2Controller megaTag2Controller = new MegaTag2Controller();

    public void initTurret(HardwareMap hardwareMap, MegaTag2Controller.targetState target) {

        limelight.init(hardwareMap);
        megaTag2Controller.initMegaTagLL(hardwareMap,target);
        targetState = target;
        pTurret.initialize(hardwareMap);


    }


    public void updateTurret(double angle, double tolerance, double offset,boolean isActivity,boolean dpad_up,boolean dpad_left,boolean x,boolean a, boolean y, boolean b,boolean stick_Button) {

        if (targetState == MegaTag2Controller.targetState.BLUE){

            mediumAngle = Math.abs(mediumAngle);
            lowAngle = Math.abs(lowAngle);
            highAngle = Math.abs(highAngle);

        }else {

            mediumAngle = -Math.abs(mediumAngle);
            lowAngle = -Math.abs(lowAngle);
            highAngle = -Math.abs(highAngle);

        }

        if (a && switchKdButton.milliseconds() > 300){

            currentModeAngle = lowAngle;
            resetSwitch();

        } else if (y && switchKdButton.milliseconds() > 300) {

            currentModeAngle  = highAngle;
            resetSwitch();

        } else if (x) {

            currentModeAngle = mediumAngle;
            resetSwitch();

        } else if (b && switchKdButton.milliseconds() > 300) {

            currentModeAngle = offsetAngle;
            resetSwitch();

        } else if (stick_Button && switchKdButton.milliseconds() > 300) {

            currentModeAngle = mediumAngle;
            resetSwitch();

        }

        if (b) {
                if (targetState == MegaTag2Controller.targetState.BLUE) {
                    megaTag2Controller.setTargetGoal(MegaTag2Controller.targetState.OFFSET_BLUE);
                } else {
                    megaTag2Controller.setTargetGoal(MegaTag2Controller.targetState.OFFSET_RED);
                }
            } else if (x || a || y) {
                if (targetState == MegaTag2Controller.targetState.BLUE) {
                    megaTag2Controller.setTargetGoal(MegaTag2Controller.targetState.BLUE);
                } else {
                    megaTag2Controller.setTargetGoal(MegaTag2Controller.targetState.RED);

            }
        }

        if(dpad_up){
            isTurretStart = true;
        }
        if (dpad_left && kdButton.milliseconds() > 300){
            isUsingHeading = !isUsingHeading;
            kdButton.reset();
        }

        if (isUsingHeading){
            Heading = angle;
        }else {
            Heading = 0;
        }

        if (isTurretStart) {


                megaTag2Controller.updateTargetGoal(0);


                if (megaTag2Controller.isValidTargetGoal()) {

                    currentAngleTx = megaTag2Controller.getTx();
                    TurretAngle = convertingPositionToAngle(pTurret.getCurrentPosition());

                    targetTx = TurretAngle - currentAngleTx;

                    lastAnglePin = Heading;

                    sumAngle = targetTx;



                } else if (!megaTag2Controller.isValidTargetGoal() && targetTx == 0){
                    lastAnglePin = Heading;
                targetTx = currentModeAngle + calcFieldAngle(Heading) + offset;
            } else {

                    currentAnglePin = Heading;
                    sumAngle = targetTx - (currentAnglePin - lastAnglePin);

                }

                }




        pTurret.update(tolerance);
        normalizedTurret = Math.max(-100,Math.min(100, currentAngleTx));

        normalizedTurret = Math.max(-100,Math.min(100, sumAngle));


        sumAngle = normalizedTurret;
        targetPosition = convertingAngleToPosition(sumAngle);



            if (isEnable) {
                pTurret.setSetPoint(targetPosition);
            }
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }


    public void setEnable(boolean state) {
        isEnable = state;
    }

    public double calcAngle(double targetX, double robotX, double robotY, double targetY) {
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        double targetAngle = Math.atan2(deltaY, deltaX);

        Math.toDegrees(targetAngle);

        return Math.toDegrees(targetAngle);
    }

    public boolean isReadyPose(double value){
        return pTurret.getErrorAbsolute() > value;
    }

    private double calcFieldAngle(double angle){
        double normalized = angle % 360;
        if (normalized > 180)  normalized -= 360;
        if (normalized <= -180) normalized += 360;

        currentAngle = normalized;
        return -currentAngle;
    }

    private double convertingAngleToPosition(double value){
        return value * DIVISIONPRICE;
    }

    private double convertingPositionToAngle(double value){
        return value / DIVISIONPRICE;
    }

    public void showTelemetry(Telemetry telemetry){

        telemetry.addLine("===Turret Controller===");
        telemetry.addData("angleGoal", angleGoal);
        telemetry.addData("is  valid apriltag", megaTag2Controller.isValidTargetGoal());
        telemetry.addData("Tx", megaTag2Controller.getTx());
        telemetry.addData("Ty", megaTag2Controller.getTy());
        telemetry.addData("y LL", megaTag2Controller.getY_MT2());
        telemetry.addData("x LL", megaTag2Controller.getX_MT2());
        telemetry.addData("comp", currentComp - lastComp);

        pTurret.showTelemetry(telemetry);

    }
    public double getXLL(){
        return megaTag2Controller.getX_MT2();
    }

    public double getYLL(){
        return megaTag2Controller.getY_MT2();
    }

    public void setTargetGoal(MegaTag2Controller.targetState targetGoal) {
        megaTag2Controller.setTargetGoal(targetGoal);
    }

    public void resetSwitch(){

        currentAngleTx = 0;
        lastAnglePin = 0;
        targetTx = 0;
        switchKdButton.reset();

    }

    }


