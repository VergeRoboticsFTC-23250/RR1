package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.CENTER;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.LEFT;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.opencv.DetectionBlue;
import org.firstinspires.ftc.teamcode.auto.paths.blue60.BlueCenter60;
import org.firstinspires.ftc.teamcode.auto.paths.blue60.BlueLeft60;
import org.firstinspires.ftc.teamcode.auto.paths.blue60.BlueRight60;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.PropPosition;

@Autonomous
@Config
public class Blue60 extends LinearOpMode {
    PropPosition position = CENTER;
    public void runOpMode() throws InterruptedException {
        DetectionBlue.init(hardwareMap, telemetry);
        Robot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Averages", DetectionBlue.pipeline.getAverages());
            telemetry.addData("Detected", DetectionBlue.getPosition());
            telemetry.update();
            position = DetectionBlue.getPosition();
            sleep(50);
        }

        waitForStart();

        if(position == LEFT){
            BlueLeft60.runAction();
        }else if (position == RIGHT){
            BlueRight60.runAction();
        }else{
            BlueCenter60.runAction();
        }
    }

}
