package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition;
import static org.firstinspires.ftc.teamcode.util.Robot.AllianceSide.*;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.LEFT;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.auto.apriltag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.opencv.DetectionRed;
import org.firstinspires.ftc.teamcode.auto.opencv.Vision;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
@Config
public class AutoRed extends LinearOpMode {
    public static int armDelay = 150;
    public static double pushIn = 3;
    public static double pushInPower = 0.5;
    public static double aprilTagMargin = 0.05;
    public static double aprilTagMinRunDuration = 3000;
    public static double headingTarget = Math.PI * 1/2;
    public static double aprilTagPGain = 4;
    PropPosition position = LEFT;
    @Config
    public static class LeftSpikeMark{
        public static Action action;
        public static double heading = -90;

        public static double x = 26;
        public static double y = -4;

        public static double x2 = 26;
        public static double y2 = 6;

        public static void buildAction(){
            action = drive.actionBuilder(drive.pose)
                    .afterTime(0.2, telemetryPacket -> {
                        Robot.Arm.setIntake();
                        Robot.Claw.setIntake();
                        return false;
                    })
                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
                    .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(heading))
                    .stopAndAdd(telemetryPacket -> {
                        Robot.Claw.setLeftGrip(true);
                        return false;
                    })
                    .build();
        }
    }

    @Config
    public static class CenterSpikeMark{
        public static Action action;
        public static double heading = -90;

        public static double x = 26;
        public static double y = -4;

        public static double x2 = 26;
        public static double y2 = 6;

        public static void buildAction(){
            action = drive.actionBuilder(drive.pose)
                    .afterTime(0.2, telemetryPacket -> {
                        Robot.Arm.setIntake();
                        Robot.Claw.setIntake();
                        return false;
                    })
                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
                    .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(heading))
                    .stopAndAdd(telemetryPacket -> {
                        Robot.Claw.setLeftGrip(true);
                        return false;
                    })
                    .build();
        }
    }

    @Config
    public static class RightSpikeMark{
        public static Action action;
        public static double heading = -90;

        public static double x = 26;
        public static double y = -4;

        public static double x2 = 26;
        public static double y2 = 6;

        public static void buildAction(){
            action = drive.actionBuilder(drive.pose)
                    .afterTime(0.2, telemetryPacket -> {
                        Robot.Arm.setIntake();
                        Robot.Claw.setIntake();
                        return false;
                    })
                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
                    .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(heading))
                    .stopAndAdd(telemetryPacket -> {
                        Robot.Claw.setLeftGrip(true);
                        return false;
                    })
                    .build();
        }
    }

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Vision.init(hardwareMap, RED);
        LeftSpikeMark.buildAction();

        initLoop(); //Detection loop
        position = LEFT;
        waitForStart();
        DetectionRed.webcam.stopStreaming();

        switch (position){
            case LEFT:
                DetectionRed.webcam.closeCameraDeviceAsync(() -> {
                    try {
                        AprilTagLocalization.init(hardwareMap, 4);
                    } catch (InterruptedException ignored) {}
                });
                Actions.runBlocking(LeftSpikeMark.action);
                break;
            case CENTER:
                break;
            case RIGHT:
                break;
        }

        boolean isOuttake = false;
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < aprilTagMinRunDuration)){
            if(System.currentTimeMillis() - startTime > armDelay && !isOuttake){
                Robot.Claw.setBothGrips(false);
                Robot.Claw.setOuttake();
                Robot.Arm.setOuttake();
                isOuttake = true;
            }
            AprilTagLocalization.update();
            drive.updatePoseEstimate();
            telemetry.addData("X       Power", AprilTagLocalization.drive);
            telemetry.addData("Y       Power", AprilTagLocalization.strafe);
            telemetry.addData("Heading Power", AprilTagLocalization.turn);
            telemetry.update();
        }

        Robot.Heading.reboot();
        double currentY = drive.pose.position.y;
        while (Math.abs(currentY - drive.pose.position.y) < pushIn){
            Robot.Chassis.run(pushInPower, 0, (headingTarget - Robot.Heading.getYaw()) * aprilTagPGain);
            telemetry.addData("status", "running Forward");
            telemetry.update();
        }

        Robot.Chassis.run(0, 0, 0);

        Robot.Claw.setBothGrips(true);
        Thread.sleep(500);
        Robot.Claw.setBothGrips(false);
        Robot.Arm.setRest();
        Robot.Claw.setRest();

        Thread.sleep(10000);
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