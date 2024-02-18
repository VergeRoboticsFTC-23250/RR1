package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
@Config
public class StraightGyro extends LinearOpMode {
    private BHI260IMU imu;
    IMU.Parameters params = new IMU.Parameters(
      new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
              RevHubOrientationOnRobot.UsbFacingDirection.UP
      )
    );

    public static double speed = 0.5;
    public static double pGain = 1;
    public static double expected = 0;
    double measured = expected;
    double error = 0;

    public void runOpMode(){
        Robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_up){
                measured = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                error = measured - expected;
                Robot.Chassis.run(speed, 0, Range.clip(error * pGain, -speed, speed));
            }else{
                Robot.Chassis.run(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            }
        }
    }
}
