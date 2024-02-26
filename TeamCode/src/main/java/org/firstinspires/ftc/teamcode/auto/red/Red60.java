package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import static org.firstinspires.ftc.teamcode.util.Robot.*;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.runX;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.runY;
import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.*;
import static org.firstinspires.ftc.teamcode.auto.red.RedParams.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.apriltag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.opencv.DetectionRed;
import org.firstinspires.ftc.teamcode.auto.opencv.Vision;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.opencv.imgproc.CLAHE;

@Autonomous
@Config
public class Red60 extends LinearOpMode {
    public static double power = 0.25;
    public static double strafeToMiddleDist = 24;
    public static double referenceForward = 0.13;
    PropPosition propPosition = PropPosition.CENTER;

    public void initLoop(){
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Detected", Vision.getPosition());
            telemetry.update();
            propPosition = Vision.getPosition();
            sleep(50);
        }
    }

    public void initAprilTag(){
        DetectionRed.webcam.stopStreaming();

        DetectionRed.webcam.closeCameraDeviceAsync(() -> {
            try {
                AprilTagLocalization.init(hardwareMap,
                        propPosition == LEFT? 4 :
                                propPosition == CENTER? 5 :
                                        propPosition == RIGHT? 6 : -1
                );
            } catch (InterruptedException ignored) {}
        });
    }

    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Vision.init(hardwareMap, AllianceSide.RED);

        initLoop();

        strafeToMiddleDist += propPosition == LEFT ? -6 : propPosition == RIGHT ? 6 : 0;

        waitForStart();

        initAprilTag();

        preload();

        Chassis.runY(strafeToMiddleDist, power, heading);

        plus2();

        Chassis.runY(strafeToMiddleDist, power, heading);

        plus2();
    }

    public void preload() throws InterruptedException {
        Vector2d position =
                propPosition == LEFT ? LeftSpikeMark.position :
                propPosition == RIGHT ? RightSpikeMark.position :
                CenterSpikeMark.position;

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.2, telemetryPacket -> {
                            Robot.Arm.setIntake();
                            Robot.Claw.setIntake();
                            return false;
                        })
                        .strafeToLinearHeading(position, Math.toRadians(heading))
                .build()
        );

        Chassis.runX(10, -power, heading);
        Claw.setRightGrip(true);
        Chassis.runX(10, power, heading);

        Claw.setBothGrips(false);
        Claw.setOuttake();

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 3000)){
                if(System.currentTimeMillis() - startTime > 500){
                Robot.Arm.setOuttake();
            }
            AprilTagLocalization.update();
            drive.updatePoseEstimate();
        }

        Chassis.runX(6, power, heading);
        Claw.setBothGrips(true);
        sleep(500);
        Claw.setBothGrips(false);
        Chassis.runX(6, -power, heading);
        Claw.setRest();
        Arm.setRest();
    }

    public void plus2() throws InterruptedException {
        //RunToBack
        Arm.setIntake();
        Claw.setOuttake();

        PIDController headingController = new PIDController(headingGains, heading);
        PIDController strafeController = new PIDController(strafeGains, drive.pose.position.x);
        while (Distance.getDist(DistanceUnit.METER) > 1){
            Robot.Chassis.run(-power, strafeController.getPower(drive.pose.position.x), headingController.getPower(Robot.Heading.getYaw()));
            drive.updatePoseEstimate();
        }

        Nicker.setOut();

        PIDController forwardController = new PIDController(new PIDCoefficients(2, 1, 0.16), referenceForward);
        while (Distance.getDist(DistanceUnit.METER) > referenceForward){
            Robot.Chassis.run(forwardController.getPower(Distance.getDist(DistanceUnit.METER)), strafeController.getPower(drive.pose.position.x), headingController.getPower(Robot.Heading.getYaw()));
            drive.updatePoseEstimate();
        }

        Claw.setIntake();

        //Intake
        while (!Color.isWhite(950)){
            Robot.Chassis.run(forwardController.getPower(Distance.getDist(DistanceUnit.METER)), -0.3, headingController.getPower(Robot.Heading.getYaw()));
            drive.updatePoseEstimate();
        }

        Claw.setBothGrips(true);

        while (opModeIsActive()){
            double activePower = -0.25;
            double standbyPower = -0.5;

            if(!Bumpers.leftSwitch.getState() && !Bumpers.rightSwitch.getState()){
                Robot.Chassis.drive.rightFront.setPower(activePower);
                Robot.Chassis.drive.rightBack.setPower(activePower);
                Robot.Chassis.drive.leftFront.setPower(activePower);
                Robot.Chassis.drive.leftBack.setPower(activePower);
            }else if(!Bumpers.leftSwitch.getState() && Bumpers.rightSwitch.getState()){
                Robot.Chassis.drive.rightFront.setPower(standbyPower);
                Robot.Chassis.drive.rightBack.setPower(standbyPower);
                Robot.Chassis.drive.leftFront.setPower(0);
                Robot.Chassis.drive.leftBack.setPower(0);
            }else if(Bumpers.leftSwitch.getState() && !Bumpers.rightSwitch.getState()){
                Robot.Chassis.drive.rightFront.setPower(0);
                Robot.Chassis.drive.rightBack.setPower(0);
                Robot.Chassis.drive.leftFront.setPower(standbyPower);
                Robot.Chassis.drive.leftBack.setPower(standbyPower);
            }else if(Bumpers.rightSwitch.getState() && Bumpers.leftSwitch.getState()){
                break;
            }

            telemetry.addData("", Bumpers.leftSwitch.getState() + " | " + Bumpers.rightSwitch.getState());
            telemetry.update();

            drive.updatePoseEstimate();
        }

        runX(0.5, power, heading);
        Nicker.setHome();
        sleep(500);
        Claw.setBothGrips(false);
        runX(6, power, heading);
        Nicker.setOut();
        runY(6, power, heading);
        Arm.setRest();
        Claw.setRest();

        //RunToForward
        double startY = drive.pose.position.y;
        strafeController.setReference(drive.pose.position.x);
        strafeController.reset();
        double startTime = System.currentTimeMillis();
        double step = 0;
        while (Math.abs(startY - drive.pose.position.y) < 68){
            if(System.currentTimeMillis() - startTime > 500 && step == 0){
                Claw.setBothGrips(true);
                step++;
            }else if(System.currentTimeMillis() - startTime > 750 && step == 1){
                Claw.setBothGrips(false);
                step++;
            }else if(System.currentTimeMillis() - startTime > 1000 && step == 2){
                Claw.setOuttake();
                step++;
            }
            Robot.Chassis.run(power, strafeController.getPower(drive.pose.position.x), headingController.getPower(Robot.Heading.getYaw()));
            drive.updatePoseEstimate();
        }

        Arm.setOuttake();

        Chassis.runY(12, -power, heading);

        //AprilTagLocalization.setDesiredTagId(-1);
        startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 3000){
            AprilTagLocalization.update();
        }

        Chassis.runX(6, power, heading);
        Claw.setBothGrips(true);
        sleep(500);
        Claw.setBothGrips(false);
        Arm.setRest();
        Claw.setRest();
        Chassis.runX(6, -power, heading);
    }
}
