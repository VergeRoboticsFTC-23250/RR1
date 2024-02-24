package org.firstinspires.ftc.teamcode.auto.tests;

import static org.firstinspires.ftc.teamcode.auto.AutoRed.activePower;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.backPower;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.backTime;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.overdrivePower;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.standbyPower;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.strafePower;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.t1;
import static org.firstinspires.ftc.teamcode.auto.AutoRed.threshhold;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
@Config
public class Intake extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Robot.RestToIntake();

        Robot.Claw.setBothGrips(true);
        DigitalChannel limitL = hardwareMap.get(DigitalChannel.class, "limitR");
        DigitalChannel limitR = hardwareMap.get(DigitalChannel.class, "limitL");

        while (!isStarted()){
            telemetry.addData("", limitL.getState() + " | " + limitR.getState());
            telemetry.addData("", Robot.Color.isWhite(threshhold));
            telemetry.update();
        }

        waitForStart();
        Robot.Claw.setBothGrips(true);

        while (opModeIsActive() && !Robot.Color.isWhite(threshhold)){
            Robot.Chassis.run(0, strafePower, 0);
            drive.updatePoseEstimate();
        }
        while (opModeIsActive()){
            telemetry.addData("", limitL.getState() + " | " + limitR.getState());
            drive.updatePoseEstimate();

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
        drive.updatePoseEstimate();
        Robot.Chassis.run(0, 0, 0);
        Robot.Nicker.setHome();
        Thread.sleep(t1 + 500);
        Robot.Claw.setBothGrips(false);
        Thread.sleep(t1);
    }
}
