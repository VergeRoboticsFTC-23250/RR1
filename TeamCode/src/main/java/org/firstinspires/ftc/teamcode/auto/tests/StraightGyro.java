package org.firstinspires.ftc.teamcode.auto.tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        waitForStart();

        double currentY = drive.pose.position.y;
        while (Math.abs(currentY - drive.pose.position.y) < pushIn){
            Robot.Chassis.run(power, 0, (target - Robot.Heading.getYaw()) * pGain);
        }
    }
}
