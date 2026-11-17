package org.firstinspires.ftc.teamcode.equipment.controller.localization;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class DriveHeadingController {

    private double lastHeading, currentHeading, setPos, heading;
    private double angleTarget;

    private double velX,velY;

    private boolean isRobotCenter = false;
    private ElapsedTime kdButton = new ElapsedTime();
    private boolean isTriangle = false;
    private GoBildaPinpointDriver pinpoint = null;
    private double lastRaw, currentRaw,deltaHeading;
    private double currentHeadingRad = 0;
    private double previousRawHeading = 0;

    private boolean wasNan = false;


    private int nanCounter = 0;
    private static final int MAX_NAN_COUNT = 5;

    private Follower follower;


    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        double initialHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        if (Double.isFinite(initialHeading)) {
            currentHeadingRad = initialHeading;
            previousRawHeading = initialHeading;
        }
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(0, 0, 0));

    }

    public void setPoseXY(Pose pose){
        follower.setPose(pose);
    }


    public void setRecalibratePinpoint() {

        pinpoint.recalibrateIMU();

    }




    public void update(double left_stick_y, double left_stick_x, double right_stick_x,boolean right_stick_button, Telemetry telemetry,boolean dpadLeft, boolean options) {
        follower.update();

        velX = follower.getVelocity().getXComponent();
        velY = follower.getVelocity().getYComponent();

        if (right_stick_button){
            pinpoint.setHeading(0,AngleUnit.RADIANS);
            pinpoint.recalibrateIMU();
            Pose bPose = new Pose(follower.getPose().getX(),follower.getPose().getY(),0);
            follower.setPose(bPose);
        }
        if (options){

            pinpoint.setHeading(0,AngleUnit.RADIANS);
            pinpoint.recalibrateIMU();
            follower.setPose(new Pose(0,0,0));

        }
        double forward = -left_stick_y;
        double strafe = -left_stick_x;
        double rotate = -right_stick_x;

        if (forward <= 0.05 && forward >= -0.05) {
            forward = 0;
        }
        if (strafe <= 0.05 && strafe >= -0.05) {
            strafe = 0;
        }
        if (rotate <= 0.05 && rotate >= -0.05) {
            rotate = 0;
        }

        follower.setTeleOpDrive(
                forward,
                strafe,
                rotate * 0.7,
                false
        );

        telemetry.addData("robot X: ", follower.getPose().getX());
        telemetry.addData("robot Y: ", follower.getPose().getY());
        telemetry.addData("robot Y: ", follower.getPose().getY());
    }
    private double normalizeAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        else if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    public int getX(){
        return (int) follower.getPose().getX();
    }
    public int getY(){
        return (int) follower.getPose().getY();
    }
    public double getAngle(){return Math.toDegrees(follower.getHeading());}

    public double getVelocityX(){return velX;}

    public double getVelocityY(){return  velY;}


}