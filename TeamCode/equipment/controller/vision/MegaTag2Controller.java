package org.firstinspires.ftc.teamcode.equipment.controller.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.equipment.implementation.Limelight;

public class MegaTag2Controller extends Limelight {
    private LLResult result;
    private double xMT2, yMT2,xMT1,yMT1,tx,ty;
    private boolean isValidTargetGoal = false,isValidPose, isDisposableValidGoal = false, isDisposableValidPose = true;

    public enum targetState{
        RED,
        BLUE,
        OFFSET_RED,
        OFFSET_BLUE,
        PREDICT_BLUE,
        PREDICT_RED,
        PREDICT_RED_BACK
    }

    private targetState targetStateCommand;

    public enum patternState{
        VIOLET_VIOLET_GREEN,
        VIOLET_GREEN_VIOLET,
        GREEN_VIOLET_VIOLET;
    }
    private static final int f_size = 5;
    private double[] xBuffer = new double[f_size];
    private double[] yBuffer = new double[f_size];
    private int bufferIndex = 0;
    private int validBufferCount = 0;

    public void initMegaTagLL(HardwareMap hardwareMap,targetState targetState) {
        super.init(hardwareMap);
        setTargetGoal(targetState);
        for (int i = 0; i < f_size; i++) {
            xBuffer[i] = 0;
            yBuffer[i] = 0;
        }
    }


    public void setTargetGoal(targetState targetState) {
        if (targetState == targetState.BLUE && targetStateCommand != MegaTag2Controller.targetState.BLUE){

            super.pipelineSwitch(0);
            targetStateCommand = MegaTag2Controller.targetState.BLUE;

        } else if (targetState == MegaTag2Controller.targetState.RED && targetStateCommand != MegaTag2Controller.targetState.RED) {

            super.pipelineSwitch(4);
            targetStateCommand = MegaTag2Controller.targetState.RED;

        } else if (targetState == MegaTag2Controller.targetState.OFFSET_RED && targetStateCommand != MegaTag2Controller.targetState.OFFSET_RED) {

            super.pipelineSwitch(3);
            targetStateCommand = MegaTag2Controller.targetState.OFFSET_RED;

        } else if (targetState == MegaTag2Controller.targetState.OFFSET_BLUE && targetStateCommand != MegaTag2Controller.targetState.OFFSET_BLUE) {
            super.pipelineSwitch(5);
            targetStateCommand = MegaTag2Controller.targetState.OFFSET_BLUE;
        } else if (targetState == MegaTag2Controller.targetState.PREDICT_RED && targetStateCommand != MegaTag2Controller.targetState.PREDICT_RED) {

            targetStateCommand = MegaTag2Controller.targetState.PREDICT_RED;
            super.pipelineSwitch(1);//67

        } else if (targetState == MegaTag2Controller.targetState.PREDICT_BLUE && targetStateCommand != MegaTag2Controller.targetState.PREDICT_BLUE) {

            super.pipelineSwitch(6);
            targetStateCommand = MegaTag2Controller.targetState.PREDICT_BLUE;

        } else if (targetState == MegaTag2Controller.targetState.PREDICT_RED_BACK && targetStateCommand != MegaTag2Controller.targetState.PREDICT_RED_BACK) {

            super.pipelineSwitch(7);
            targetStateCommand = MegaTag2Controller.targetState.PREDICT_RED_BACK;

        }
    }

    public void updateTargetGoal(double angle) {
        super.updateRobotOrientation(angle);
        result = super.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            isValidTargetGoal = true;
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            Pose3D botpose_mt = result.getBotpose();
            if (botpose_mt2 != null) {
                double currentX = botpose_mt2.getPosition().x;
                double currentY = botpose_mt2.getPosition().y;
                xBuffer[bufferIndex] = currentX;
                yBuffer[bufferIndex] = currentY;
                bufferIndex = (bufferIndex + 1) % f_size;
                if (validBufferCount < f_size) {
                    validBufferCount++;
                }
                double sumX = 0;
                double sumY = 0;
                for (int i = 0; i < validBufferCount; i++) {
                    sumX += xBuffer[i];
                    sumY += yBuffer[i];
                }
                this.xMT2 = sumX / validBufferCount;
                this.yMT2 = sumY / validBufferCount;
                isValidPose = true;
            }else {
                isValidPose = false;
            }

            if (botpose_mt != null){
                xMT1 = botpose_mt.getPosition().x;
                yMT1 = botpose_mt.getPosition().y;
            }


        }else {
            isValidTargetGoal = false;
            isValidPose = false;
            validBufferCount = 0;
            bufferIndex = 0;
        }
    }

    public void updateTx(){

    }

    public boolean isValidTargetGoal(){
        return isValidTargetGoal;
    }

    public double getTx(){
        return tx;
    }
    public double getTy(){
        return ty;
    }
    public boolean isValidPose(){
        return isValidPose;
    }
    public double getX_MT2(){
        return xMT2;
    }
    public double getY_MT2(){
        return yMT2;
    }

    public double getX_MT(){
        return xMT1;
    }

    public double getY_MT(){
        return yMT1;
    }

    public boolean isDisposableValidGoal(){
        return isDisposableValidGoal;
    }

    public boolean isDisposableValidPose(){
        return isDisposableValidPose;
    }

    public void showTelemetry(Telemetry telemetry){

        telemetry.addLine("===Limelight3A-MegaTag2===");

        telemetry.addData("is Valid apriltag: ",  isValidTargetGoal);
        telemetry.addData("Tx: ", getTx());
        telemetry.addData("Ty" , getTy());
        telemetry.addData("Is valid pose: ", isValidPose);
        telemetry.addData("X: " , xMT2);
        telemetry.addData("Y: ", yMT2);

    }

}