package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.CENTER;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.LEFT;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.opencv.DetectionBlue;
import org.firstinspires.ftc.teamcode.auto.opencv.DetectionRed;
import org.firstinspires.ftc.teamcode.auto.paths.blue60.BlueRight60;
import org.firstinspires.ftc.teamcode.auto.paths.red60.RedCenter60;
import org.firstinspires.ftc.teamcode.auto.paths.red60.RedLeft60;
import org.firstinspires.ftc.teamcode.auto.paths.red60.RedRight60;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.PropPosition;

@Autonomous
@Config
public class Red60 extends LinearOpMode {
    PropPosition position = CENTER;
    public void runOpMode() throws InterruptedException {
        DetectionRed.init(hardwareMap, telemetry);
        Robot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Averages", DetectionRed.pipeline.getAverages());
            telemetry.addData("Detected", DetectionRed.getPosition());
            telemetry.update();
            position = DetectionRed.getPosition();
            sleep(50);
        }

        waitForStart();

        if(position == LEFT){
            RedLeft60.runAction();
        }else if (position == RIGHT){
            RedRight60.runAction();
        }else{
            RedCenter60.runAction();
        }
    }

}
