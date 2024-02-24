package org.firstinspires.ftc.teamcode.auto.tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
@Config
public class StraightGyro extends LinearOpMode {
    public static double pushIn = 5;
    public static double power = 0.25;
    public static double target = Math.PI * 1/2;
    public static double pGain = 4;
    public void runOpMode(){
        Robot.init(hardwareMap);
        DigitalChannel limitL = hardwareMap.get(DigitalChannel.class, "limitR");
        DigitalChannel limitR = hardwareMap.get(DigitalChannel.class, "limitL");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("", limitL.getState() + " | " + limitR.getState());
            telemetry.update();
        }
    }
}
