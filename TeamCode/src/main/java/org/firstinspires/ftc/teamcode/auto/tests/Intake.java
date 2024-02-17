package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
@Config
public class Intake extends LinearOpMode {
    public static double activePower = -0.25;
    public static double overdrivePower = -0.5;
    public static double standbyPower = 0;
    public static double backPower = 0.15;
    public static int backTime = 150;
    public static double threshhold = 500;
    public static double strafePower = 0.3;
    public static int t1 = 250;
    public static int regriptime = 250;
    public static int regripDelay = 500;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.RestToIntake();
        Robot.Claw.setBothGrips(true);
        DigitalChannel limitL = hardwareMap.get(DigitalChannel.class, "limitR");
        DigitalChannel limitR = hardwareMap.get(DigitalChannel.class, "limitL");

        while (!isStarted()){
            telemetry.addData("", limitL.getState() + " | " + limitR.getState());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive() && !Robot.Color.isWhite(threshhold)){
            Robot.Chassis.run(0, strafePower, 0);
        }
        while (opModeIsActive()){
            telemetry.addData("", limitL.getState() + " | " + limitR.getState());

            if(!limitL.getState() && !limitR.getState()){
                Robot.Chassis.drive.rightFront.setPower(activePower);
                Robot.Chassis.drive.rightBack.setPower(activePower);
                Robot.Chassis.drive.leftFront.setPower(activePower);
                Robot.Chassis.drive.leftBack.setPower(activePower);
            }else if(!limitL.getState() && limitR.getState()){
                Robot.Chassis.drive.rightFront.setPower(standbyPower);
                Robot.Chassis.drive.rightBack.setPower(standbyPower);
                Robot.Chassis.drive.leftFront.setPower(overdrivePower);
                Robot.Chassis.drive.leftBack.setPower(overdrivePower);
            }else if(limitL.getState() && !limitR.getState()){
                Robot.Chassis.drive.rightFront.setPower(overdrivePower);
                Robot.Chassis.drive.rightBack.setPower(overdrivePower);
                Robot.Chassis.drive.leftFront.setPower(standbyPower);
                Robot.Chassis.drive.leftBack.setPower(standbyPower);
            }else if(limitR.getState() && limitL.getState()){
                break;
            }
            telemetry.update();
        }
        Robot.Chassis.run(backPower, 0, 0);
        Thread.sleep(backTime);
        Robot.Chassis.run(0, 0, 0);
        Robot.Nicker.setHome();
        Thread.sleep(t1);
        Robot.Claw.setBothGrips(false);
        Thread.sleep(t1);
        Robot.Nicker.setOut();
        Thread.sleep(t1);
        Robot.Claw.setRest();
        Robot.Arm.setRest();
        Thread.sleep(regripDelay);
        Robot.Claw.setBothGrips(true);
        Thread.sleep(regriptime);
        Robot.Claw.setBothGrips(false);
        while (opModeIsActive()){

        }
    }
}
