package org.firstinspires.ftc.teamcode.equipment.implementation;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Limelight {
    private Limelight3A limelight;
    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
    }
    public void pipelineSwitch(int index){
        limelight.pipelineSwitch(index);
    }
    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }
    public void updateRobotOrientation(double yaw){
        limelight.updateRobotOrientation(yaw);

    }


}

