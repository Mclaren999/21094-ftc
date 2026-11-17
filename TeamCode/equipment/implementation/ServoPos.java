package org.firstinspires.ftc.teamcode.equipment.implementation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ServoPos {
    private Servo shooterServo;


    public void initServo(HardwareMap hw){

        shooterServo = hw.get(Servo.class,"shooter_Servo");//shooter_Servo
        shooterServo.setDirection(Servo.Direction.REVERSE);

    }

    public void setPosServo(double position){
        shooterServo.setPosition(position);
    }
    public double getServoPos() {
        return shooterServo.getPosition();
    }
    //13, -0,5

}
