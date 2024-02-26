package org.firstinspires.ftc.teamcode.auto.tests;

import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
@Config
public class StraightGyro extends LinearOpMode {
    public static double target = 0;
    public static double kP = 2;
    public static double kI = 0;
    public static double kD = 0;
    public static double power = 0;
    public static double dist = 12;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);

        waitForStart();

        double currentX = drive.pose.position.x;
        double currentY = drive.pose.position.y;
        PIDController headingController = new PIDController(2, 0, 0.001, target);
        PIDController yController = new PIDController(kP, kI, kD, currentY);
        while (Math.abs(currentX - drive.pose.position.x) < dist){
            Robot.Chassis.run(power, yController.getPower(drive.pose.position.y), headingController.getPower(Robot.Heading.getYaw()));
            yController.setGains(kP, kI, kD);
            drive.updatePoseEstimate();
        }
        Robot.Chassis.run(0, 0, 0);
    }
}
