package org.firstinspires.ftc.teamcode.auto.tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.Robot;

@Config
@TeleOp
public class Test extends LinearOpMode {
    DigitalChannel rightSwitch;
    DigitalChannel leftSwitch;

    public void runOpMode(){
        rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");
        leftSwitch = hardwareMap.get(DigitalChannel.class, "leftSwitch");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Right", rightSwitch.getState());
            telemetry.addData("Left", leftSwitch.getState());
            telemetry.update();
        }
    }
}
