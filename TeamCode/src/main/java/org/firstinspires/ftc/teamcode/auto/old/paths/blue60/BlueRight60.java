package org.firstinspires.ftc.teamcode.auto.old.paths.blue60;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Config
@Autonomous
@Disabled
public class BlueRight60 extends LinearOpMode {
    public static double x1 = 60;
    public static double y1 = 5;

    public static double delay1 = 0.85;

    public static double x2 = 60;
    public static double y2 = -7.5;

    public static double delay2 = 0.2;

    public static double x3 = 78;
    public static double y3 = 82;

    public static double delay3 = 0.5;

    public static double x4 = 110;
    public static double y4 = 0;

    public static double x5 = 110;
    public static double y5 = -138;

    public static double parkX = -24;

    public static double powerY = 0.30;
    public static double powerH = -0.005;

    public static double powerX = 0.2;
    public static double powerStrafe = -0.2;
    public static double threshhold = 300;
    public static int pullTime = 150;
    public static int pushTime = 1500;
    public static int strafeTime = 200;

    public static double offsetX = -2;
    public static double offsetY = -6;
    public static double parkBack = 8;

    public static double slides = 750;
    public static double slidesPower = 500;

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        waitForStart();
        runAction();
    }

    public static void runAction() throws InterruptedException {
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .afterTime(delay1, telemetryPacket -> {
                            Robot.Claw.setIntake();
                            return false;
                        })
                        .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(90))

                        .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(90))
                        .stopAndAdd(telemetryPacket -> {
                            Robot.Claw.setRightGrip(true);
                            return false;
                        })
                        .afterTime(delay2, telemetryPacket -> {
                            Robot.Claw.setBothGrips(false);
                            Robot.Claw.setOuttake();
                            Robot.Arm.setOuttake();
                            return false;
                        })
                        .strafeToLinearHeading(new Vector2d(x3, y3), Math.toRadians(90))
                        .stopAndAdd(telemetryPacket -> {
                            Robot.Claw.setBothGrips(true);
                            return false;
                        })
                        .afterTime(delay3, telemetryPacket -> {
                            Robot.Claw.setRest();
                            Robot.Claw.setBothGrips(false);
                            Robot.Arm.setRest();
                            return false;
                        })
//                        .strafeToLinearHeading(new Vector2d(x4, y4), Math.toRadians(90))
//                        .afterTime(0.25, telemetryPacket -> {
//                            Robot.Nicker.setOut();
//                            return false;
//                        })
//                        .afterTime(0.5, telemetryPacket -> {
//                            Robot.Claw.setIntake();
//                            Robot.Arm.setIntake();
//                            return false;
//                        })
//                        .afterTime(1, telemetryPacket -> {
//                            Robot.Claw.setBothGrips(true);
//                            return false;
//                        })
//                        .strafeToLinearHeading(new Vector2d(x5, y5), Math.toRadians(90))
//                        .stopAndAdd(telemetryPacket -> {
//                            if (Robot.Color.isWhite(threshhold)) {
//                                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                                drive.updatePoseEstimate();
//                                return false;
//                            }else{
//                                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, powerY), powerH));
//                                drive.updatePoseEstimate();
//                                return true;
//                            }
//                        })
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(x3, y3 - parkBack), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(parkX, y3 - parkBack), Math.toRadians(90))
                        .build()
        );

//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, powerStrafe), powerH));
//        Thread.sleep(strafeTime);
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-powerX, 0), 0));
//        Thread.sleep(pushTime);
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(powerX, 0), 0));
//        Thread.sleep(pullTime);
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//        Robot.Nicker.setHome();
//        Thread.sleep(750);
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .afterTime(0.5, telemetryPacket -> {
//                            Robot.Claw.setBothGrips(false);
//                            return false;
//                        })
//                        .afterTime(0.75, telemetryPacket -> {
//                            Robot.Nicker.setOut();
//                            return false;
//                        })
//                        .afterTime(1, telemetryPacket -> {
//                            Robot.Arm.setRest();
//                            Robot.Claw.setRest();
//                            return false;
//                        })
//                        .afterTime(1.5, telemetryPacket -> {
//                            Robot.Claw.setBothGrips(true);
//                            return false;
//                        })
//                        .afterTime(2.0, telemetryPacket -> {
//                            Robot.Claw.setBothGrips(false);
//                            return false;
//                        })
//                        .splineTo(new Vector2d(x4, y4), Math.toRadians(90))
//                        .afterTime(0.5, telemetryPacket -> {
//                            Robot.Arm.setOuttake();
//                            Robot.Claw.setOuttake();
//                            return false;
//                        })
//                        .splineTo(new Vector2d(x3 - offsetX, y3 + offsetY), Math.toRadians(90))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(telemetryPacket -> {
//                            Robot.Claw.setBothGrips(true);
//                            return false;
//                        })
//                        .waitSeconds(0.5)
//                        .stopAndAdd(telemetryPacket -> {
//                            Robot.Claw.setBothGrips(false);
//                            Robot.Claw.setRest();
//                            Robot.Arm.setRest();
//                            Robot.Nicker.setRest();
//                            return false;
//                        })
//                        .strafeToLinearHeading(new Vector2d(x3 - offsetX, y3 + offsetY - parkBack), Math.toRadians(90))
//                        .strafeToLinearHeading(new Vector2d(parkX, y3 + offsetY - parkBack), Math.toRadians(90))
//                        .build()
//        );
    }
}
