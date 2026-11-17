package org.firstinspires.ftc.teamcode.equipment.controller.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class TurretController_Chpenya {
        private double DIVISIONPRICE = 2.688888888888889;

        private double medFlightSpeed;

        private double predictionX,predictionY;

        private boolean isManual, isHeading = true;

        private ElapsedTime kdManualSwitch = new ElapsedTime(), kdButtonHeading = new ElapsedTime();

        public static double manualAngle, lowAngle,mediumAngle,highAngle;

        public static double sX,sY;

        public static double DIVISION_COORDINATE = 0.001;
        private double normalizedTurret;

        public static double angleMedium = -45;

        private double targetPosition;
        private double currentAngle;
        private double fieldAngle;
        private double calcAngle;
        private double sumAngle;
        private boolean isEnable = true;
        private PTurret pTurret = new PTurret();
        private double fAngle;
        private double offset = 0;

        public enum targetTeam{
            BLUE,
            RED;
        }

        private double currentPositionAngle;
        public void init(HardwareMap hardwareMap){
            pTurret.initialize(hardwareMap);
        }

        public void turretUpdate(double tolerance,double angle,double x1, double x2 , double y1 , double y2, double timePrediction,double velX,double velY,double toleranceVel,boolean dpad_down,boolean a,boolean x, boolean y,boolean dpad_left,double offset,targetTeam targetState){

            pTurret.update(tolerance);

            if (targetState == targetTeam.BLUE){
                mediumAngle = Math.abs(mediumAngle);
                lowAngle = Math.abs(lowAngle);
                highAngle = Math.abs(highAngle);
            }else {
                mediumAngle = -Math.abs(mediumAngle);
                lowAngle = -Math.abs(lowAngle);
                highAngle = -Math.abs(highAngle);
            }

                compensationMove(timePrediction, velX, velY, toleranceVel,x1,x2,y1,y2);
                calcAngle = calcAngle(x2, x1 + predictionX, y1 + predictionY, y2);
                sumAngle = fieldTurret(angle) + calcAngle;

            normalizedTurret = Math.max(-100,Math.min(100, sumAngle));


            sumAngle = normalizedTurret;
            targetPosition = sumAngle * DIVISIONPRICE;



                if (isEnable) {

                pTurret.setTargetPosition(targetPosition);


                }

            }


        private double fieldTurret(double angle) {
            double normalized = angle % 360;
            if (normalized > 180)  normalized -= 360;
            if (normalized <= -180) normalized += 360;

            currentAngle = normalized;
            return -currentAngle;
        }

        private void updateManualTurret(boolean a,boolean x,boolean y, double offset){

            if(x){
                manualAngle = mediumAngle;
            } else if (y) {
                manualAngle = highAngle;
            } else if (a) {
                manualAngle = lowAngle;
            }
            sumAngle = fieldAngle + manualAngle + offset;

            normalizedTurret = sumAngle % 360;

            if (normalizedTurret > 180) {
                normalizedTurret -= 360;
            }
            if (normalizedTurret < -180) {
                normalizedTurret += 360;
            }

            targetPosition = convertingAngle(normalizedTurret);

            pTurret.setTargetPosition(targetPosition);
        }

        public void compensationMove(double timePrediction,double velX, double velY, double toleranceVel,double x1Pin,double x2Pin,double y1Pin, double y2Pin){

            if (velX < toleranceVel && velY < toleranceVel) {
                sX = 0;
                sY = 0;
                return;
            }
             sX = velX * timePrediction;
             sY =  velY * timePrediction ;

             double distance = Math.hypot(x2Pin - x1Pin, y2Pin - y1Pin);


            predictionX = sX /* + (distance / medFlightSpeed) * velX  */ ;
             predictionY = sY  /* + (distance / medFlightSpeed) * velY  */;

        }

        public double getFieldAngle(){
            return currentAngle;
        }

        public double calcAngle(double targetX, double robotX, double robotY , double targetY) {

            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;

            double targetAngle = Math.atan2(deltaY, deltaX);

            return Math.toDegrees(targetAngle);
        }

        public double setOffset(double offset){
            this.offset = offset;
            return offset;
        }

        public void setEnable(boolean isEnable){
            this.isEnable = isEnable;
        }

        public double getSumAngle(){
            return sumAngle;
        }

        public double getOffset(){
            return offset;
        }

        private double convertingAngle(double value){
            return value * DIVISIONPRICE;
        }

        //Manual


        public boolean isReadyPose(double value){
            return pTurret.getErrorAbsolute() > value;

        }

        public void showTelemetry(Telemetry telemetry) {
            telemetry.addLine("=== Turret Controller ===");
            telemetry.addData("fieldAngle", getFieldAngle());
            telemetry.addData("sumAngle", getSumAngle());
            telemetry.addData("current offset: ", getOffset());
            telemetry.addData("targetPosition" , targetPosition);
            telemetry.addData("sX", sX);
            telemetry.addData("sY", sY);
//            pTurret.showTelemetry(telemetry);
        }

    }

