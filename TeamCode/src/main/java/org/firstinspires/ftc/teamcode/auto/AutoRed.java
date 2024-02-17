package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition;
import static org.firstinspires.ftc.teamcode.util.Robot.AllianceSide.*;

import org.firstinspires.ftc.teamcode.auto.apriltag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.opencv.Vision;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
@Config
public class AutoRed extends LinearOpMode {
    PropPosition position = PropPosition.LEFT;

    static Pose2d startPose = new Pose2d(9, -24, 180);

    public static class LeftSpikeMark{
        public static Action action;
        public static double x = 33;
        public static double y = 4;
        public static double offset = 1;

        public static void buildAction(){
            action = drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(x, startPose.position.y - offset), 90)
                    .strafeToLinearHeading(new Vector2d(x, y), 90)
                    .build();
        }
    }

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Vision.init(hardwareMap, RED);

        LeftSpikeMark.buildAction();

        //initLoop(); //Detection loop
        waitForStart();

        switch (position){
            case LEFT:
                AprilTagLocalization.init(hardwareMap, 4);
                Actions.runBlocking(LeftSpikeMark.action);
                while (opModeIsActive()){
                    AprilTagLocalization.update();
                    telemetry.addData("X       Error", AprilTagLocalization.getXError());
                    telemetry.addData("Y       Error", AprilTagLocalization.getYError());
                    telemetry.addData("Heading Error", AprilTagLocalization.getHeadingError());
                    telemetry.update();
                }
                break;
            case CENTER:
                break;
            case RIGHT:
                break;
        }
    }

    public void initLoop(){
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Detected", Vision.getPosition());
            telemetry.update();
            position = Vision.getPosition();
            sleep(50);
        }
    }
}