package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.roadrunner.ThreeDeadWheelLocalizer;

@Autonomous
@Config
public class RobotHeadingFunctions extends LinearOpMode {
    public static double KpForward = 2;
    public static double KiForward = 1;
    public static double KdForward = 0.16;

    public static double KpHeading = 2;
    public static double KiHeading = 0;
    public static double KdHeading = 0.001;

    public static double referenceForward = 0.185;
    public static double referenceHeading = Math.PI;
    public static double takeoverDist = 0.8;
    public static double power = -1;
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        waitForStart();
        ElapsedTime timerHeading = new ElapsedTime();

        double integralSumHeading = 0;
        double lastErrorHeading = 0;

        double dist = Robot.Distance.getDist(DistanceUnit.METER);

        while (opModeIsActive() && dist > takeoverDist){
            double heading = Robot.Heading.getYaw();
            double errorHeading = referenceHeading - heading;
            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();
            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());
            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);

            dist = Robot.Distance.getDist(DistanceUnit.METER);

            telemetry.addData("dist", dist);
            telemetry.update();

            Robot.Chassis.run(power, 0, outHeading);

            lastErrorHeading = errorHeading;
            timerHeading.reset();
        }

        double integralSumForward = 0;

        double lastErrorForward = 0;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double distance = Robot.Distance.getDist(DistanceUnit.METER);
            double heading = Robot.Heading.getYaw();

            double errorForward = referenceForward - distance;
            double errorHeading = referenceHeading - heading;

            double derivativeForward = (errorForward - lastErrorForward) / timer.seconds();
            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();

            integralSumForward = integralSumForward + (errorForward * timer.seconds());
            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());

            double outForward = (KpForward * errorForward) + (KiForward * integralSumForward) + (KdForward * derivativeForward);
            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);

            Robot.Chassis.run(outForward, 0, outHeading);

            lastErrorForward = errorForward;
            lastErrorHeading = errorHeading;

            timer.reset();
            timerHeading.reset();

            telemetry.addData("dist", distance);
            telemetry.update();
        }
    }

    double getPos(){
        return ThreeDeadWheelLocalizer.par0.getPositionAndVelocity().position * MecanumDrive.PARAMS.inPerTick;
    }
}
