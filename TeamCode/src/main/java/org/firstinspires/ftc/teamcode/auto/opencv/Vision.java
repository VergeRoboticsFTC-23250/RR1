package org.firstinspires.ftc.teamcode.auto.opencv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Robot;

public class Vision {
    static Robot.AllianceSide side = null;
    public static void init(HardwareMap hardwareMap, Robot.AllianceSide side){
        Vision.side = side;
        if(side == Robot.AllianceSide.BLUE){
            DetectionBlue.init(hardwareMap);
        }else{
            DetectionRed.init(hardwareMap);
        }
    }

    public static String getAverages(){
        if(side == Robot.AllianceSide.BLUE){
            return DetectionBlue.pipeline.getAverages();
        }else{
            return DetectionRed.pipeline.getAverages();
        }
    }

    public static Robot.PropPosition getPosition(){
        if(side == Robot.AllianceSide.BLUE){
            return DetectionBlue.getPosition();
        }else{
            return DetectionRed.getPosition();
        }
    }
}
