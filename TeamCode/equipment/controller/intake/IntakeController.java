package org.firstinspires.ftc.teamcode.equipment.controller.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeController {

    private ElapsedTime kdButtonL2 = new ElapsedTime();
    private IntakeState intakeState = IntakeController.IntakeState.NULL;
    private boolean isCollection = false;


    public void initMotor(HardwareMap hwmap) {
        intakeOne = hwmap.get(DcMotor.class, "intake");
        intakeOne.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void initServo(HardwareMap hwmap){
        intakeStopper = hwmap.get(Servo.class,"opener");
    }

    public void intakeUpdate(double rightTrigger , boolean rightBumper, double leftTrigger, boolean b, boolean isActive, Gamepad gamepad){
        if (rightBumper && isActive){

            isCollection = false;
            setIntakeOne(1);
            setPoseOpenStoper();
            intakeState = intakeState.SHOOT;

            gamepad.rumble(1.0, 0.0, 70);

        } else if (rightTrigger > 0 && kdButtonL2.milliseconds() > 500) {
            kdButtonL2.reset();
            isCollection = !isCollection;

            setIntakeOne(1);
            setCloseStoper();
            intakeState = intakeState.COLLECTION;

        }

         if (leftTrigger > 0){
            isCollection = false;
            setIntakeOne(-1);
            setCloseStoper();intakeState = intakeState.OUTTAKE;

        }

        if (leftTrigger == 0 && !rightBumper && !isCollection && !b) {
            setIntakeOne(0);
            setCloseStoper();
            intakeState = intakeState.STOP;

        }

    }

    public IntakeState getState(){
        return intakeState;
    }
    public void showTelemetry(Telemetry telemetry){
        telemetry.addLine("=== Intake Controller ===");
        telemetry.addData("StateIntake: ", getState());
    }

    private  double OPEN_STOPER = 0.75;
    private double CLOSE_STOPER = 0.5;
    private DcMotor intakeOne;
    private Servo intakeRamp, intakeStopper;



    public void setPoseOpenStoper(){
        intakeStopper.setPosition(OPEN_STOPER);
    }
    public void setCloseStoper(){
        intakeStopper.setPosition(CLOSE_STOPER);
    }

    public enum IntakeState {
        NULL,
        SHOOT,
        COLLECTION,
        OUTTAKE,
        RESET,
        STOP;

        @Override
        public String toString() {
            switch (this) {
                case STOP:
                    return "🛑 Stopped";
                case COLLECTION:
                    return "📥 Collecting";
                case SHOOT:
                    return "🎯 Shooting";
                case RESET:
                    return "🔄 Resetting";
                case OUTTAKE:
                    return "⏪ Outtake";
                default:
                    return "Null";

            }


        }
    }

    public void setIntakeOne(double power) {
        intakeOne.setPower(power);


    }

}
