package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.roadrunner.TwoDeadWheelLocalizer;

@Config
public class Robot {
    public static double SLOW_SPEED = 0.3;
    public static int SLEEP_TIME_SHORT = 300;
    public static int SLEEP_TIME_LONG = 700;

    public static double pGainHeading = 4;

    public enum RobotState{INTAKE, REST, OUTTAKE}

    public enum PropPosition{
        LEFT,
        CENTER,
        RIGHT
    }

    public enum AllianceSide{
        BLUE,
        RED,
    }

    public static RobotState robotState = RobotState.INTAKE;

    public static void init(HardwareMap hardwareMap){
        Chassis.init(hardwareMap);
        Slides.init(hardwareMap);
        Hook.init(hardwareMap);
        Airplane.init(hardwareMap);
        Arm.init(hardwareMap);
        Claw.init(hardwareMap);
        Nicker.init(hardwareMap);
        LimitSwitch.init(hardwareMap);
        Color.init(hardwareMap);
        Heading.init(hardwareMap);
        Distance.init(hardwareMap);
    }

    public static void RestToIntake() throws InterruptedException {
        Slides.run(0);
        Claw.setBothGrips(false);
        Nicker.setOut();
        Arm.setIntake();
        Thread.sleep(SLEEP_TIME_SHORT);
        Claw.setIntake();
        Thread.sleep(SLEEP_TIME_SHORT);

        robotState = RobotState.INTAKE;
    }

    public static void IntakeToRest() throws InterruptedException {
        Slides.run(0);
        Claw.setBothGrips(false);
        Nicker.setRest();
        Claw.setRest();
        Arm.setRest();
        Thread.sleep(SLEEP_TIME_SHORT);

        robotState = RobotState.REST;
    }

    public static void RestToOuttake() throws InterruptedException {
        Slides.run(0);
        Claw.setBothGrips(false);
        Claw.setOuttake();
        Thread.sleep(SLEEP_TIME_SHORT);
        Arm.setOuttake();
        Thread.sleep(SLEEP_TIME_SHORT);

        robotState = RobotState.OUTTAKE;
    }

    public static void OuttakeToRest() throws InterruptedException {
        Slides.run(0);
        Claw.setBothGrips(false);
        Claw.setRest();
        Thread.sleep(SLEEP_TIME_SHORT);
        Arm.setRest();
        Thread.sleep(SLEEP_TIME_LONG);
        Claw.setRest();
        Thread.sleep(SLEEP_TIME_SHORT);

        robotState = RobotState.REST;
    }

    public static class Chassis{
        public static MecanumDrive drive;

        public static void init(HardwareMap hardwareMap){
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        }

        public static void init(HardwareMap hardwareMap, Pose2d pose){
            drive = new MecanumDrive(hardwareMap, pose);
        }

        public static void run(double x, double y, double heading){
            drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(x, y), heading)
            );

            drive.updatePoseEstimate();
        }

        public static void runX(double dist, double power) throws InterruptedException {
            Robot.Heading.reboot();
            double currentY = drive.pose.position.y;
            while (Math.abs(currentY - drive.pose.position.y) < dist){
                Robot.Chassis.run(power, 0, ((Math.PI * 1/2) - Robot.Heading.getYaw()) * pGainHeading);
                drive.updatePoseEstimate();
            }
            Robot.Chassis.run(0, 0, 0);
        }

        public static void runY(double dist, double power) throws InterruptedException {
            Robot.Heading.reboot();
            double currentX = drive.pose.position.x;
            while (Math.abs(currentX - drive.pose.position.x) < dist){
                Robot.Chassis.run(0, power, ((Math.PI * 1/2) - Robot.Heading.getYaw()) * pGainHeading);
                drive.updatePoseEstimate();
            }
            Robot.Chassis.run(0, 0, 0);
        }
    }

    @Config
    public static class Slides{
        public static DcMotor leftSlideMotor;
        public static DcMotor rightSlideMotor;

        public static int MAX = 2300;
        public static int MIN = 100;
        public static void init(HardwareMap hardwareMap){
            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlide");

            leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public static int getPos(){
            return (int)Math.round((leftSlideMotor.getCurrentPosition() + rightSlideMotor.getCurrentPosition()) / 2.0);
        }

        public static int getPosDifferance(){
            return leftSlideMotor.getCurrentPosition() - rightSlideMotor.getCurrentPosition();
        }

        public static String getPosIndividual(){
            return "Left: " + leftSlideMotor.getCurrentPosition() + "Right: " + rightSlideMotor.getCurrentPosition();
        }

        public static boolean isBusy(){
            return leftSlideMotor.isBusy() && rightSlideMotor.isBusy();
        }

        public static void run(double pow){
            int pos = getPos();

            if(pos < MAX && pos > MIN || pos >= MAX && pow <= 0 || pos <= MIN && pow >= 0){
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftSlideMotor.setPower(pow);
                rightSlideMotor.setPower(pow);
            }else{
                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);
            }
        }

        public static boolean run(int pos, double pow){
            leftSlideMotor.setPower(0);
            rightSlideMotor.setPower(0);

            leftSlideMotor.setTargetPosition(pos);
            rightSlideMotor.setTargetPosition(pos);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideMotor.setPower(pow);
            rightSlideMotor.setPower(pow);

            if(isBusy()){
                return true;
            }else{
                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);
                return false;
            }
        }

        public static String getTargetPositions(){
            return "Left Side: " + leftSlideMotor.getTargetPosition() + " Right Side: " + rightSlideMotor.getTargetPosition();
        }
    }

    @Config
    public static class Arm{
        public static Servo rightPivot;
        public static Servo leftPivot;

        public static double restPos = 0.35;
        public static double outtakePos = 0.86;
        public static double intakePos = 0.3;
        public static double offset = 0;

        public static boolean isResting = false;

        public static void init(HardwareMap hardwareMap){
            rightPivot = hardwareMap.get(Servo.class, "rightArm");
            leftPivot = hardwareMap.get(Servo.class, "leftArm");

            leftPivot.setDirection(Servo.Direction.REVERSE);

            setRest();
        }

        public static void setIntake(){
            rightPivot.setPosition(intakePos + offset);
            leftPivot.setPosition(intakePos);
        }

        public static void setRest(){
            rightPivot.setPosition(restPos + offset);
            leftPivot.setPosition(restPos);
            isResting = true;
        }

        public static void setOuttake(){
            rightPivot.setPosition(outtakePos + offset);
            leftPivot.setPosition(outtakePos);
            isResting = false;
        }
    }

    @Config
    public static class Claw{
        public static Servo pivot;
        public static Servo rightGrip;
        public static Servo leftGrip;

        public static double openRight = 0.40;
        public static double openLeft = 0.35;
        public static double closePos = 0.5;

        public static double intakePos = 0.65;
        public static double restPos = 0.125;
        public static double outtakePos = 0.325;
        public static double safePos = 0.325;

        public static void init(HardwareMap hardwareMap){
            pivot = hardwareMap.get(Servo.class, "clawPivot");

            rightGrip = hardwareMap.get(Servo.class, "rightGrip");
            leftGrip = hardwareMap.get(Servo.class, "leftGrip");

            leftGrip.setDirection(Servo.Direction.REVERSE);

            setRest();
            setBothGrips(false);
        }

        public static void setLeftGrip(boolean state){
            leftGrip.setPosition(state? openLeft : closePos);
        }

        public static void setRightGrip(boolean state){
            rightGrip.setPosition(state? openRight : closePos);
        }

        public static void setBothGrips(boolean state){
            setLeftGrip(state);
            setRightGrip(state);
        }

        public static void setIntake(){
            pivot.setPosition(intakePos);
        }

        public static void setRest(){
            pivot.setPosition(restPos);
        }

        public static void setOuttake(){
            pivot.setPosition(outtakePos);
        }

        public static void setSafe(){
            pivot.setPosition(safePos);
        }
    }

    @Config
    public static class Nicker{
        public static Servo rightNicker;
        public static Servo leftNicker;

        public static double homeR = .3;
        public static double homeL = .3;
        public static double outL = 0.9;
        public static double outR = 0.965;
        public static double rest = 0.025;

        public static void init(HardwareMap hardwareMap){
            rightNicker = hardwareMap.get(Servo.class, "rightNicker");
            leftNicker = hardwareMap.get(Servo.class, "leftNicker");

            leftNicker.setDirection(Servo.Direction.REVERSE);

            setRest();
        }

        public static void setHome(){
            rightNicker.setPosition(homeR);
            leftNicker.setPosition(homeL);
        }

        public static void setOut(){
            rightNicker.setPosition(outR);
            leftNicker.setPosition(outL);
        }

        public static void setRest(){
            rightNicker.setPosition(rest);
            leftNicker.setPosition(rest);
        }

        public static void setRightRest(){
            rightNicker.setPosition(rest);
        }

        public static void setLeftRest(){
            leftNicker.setPosition(rest);
        }

        public static void setRightHome(){
            rightNicker.setPosition(homeR);
        }

        public static void setLeftHome(){
            leftNicker.setPosition(homeL);
        }

        public static void setRightOut(){
            rightNicker.setPosition(outR);
        }

        public static void setLeftOut(){
            leftNicker.setPosition(outL);
        }
    }

    @Config
    public static class Hook{
        public static int extendPos = -6000;
        private static DcMotor hookMotor;

        private static boolean isExtended = false;

        public static void init(HardwareMap hardwareMap){
            hookMotor = hardwareMap.get(DcMotor.class, "hook");
            hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public static void toggle(int pow){
            hookMotor.setPower(0);
            isExtended = !isExtended;
            hookMotor.setTargetPosition(isExtended? extendPos : -1500);
            hookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hookMotor.setPower(pow);
            while (hookMotor.isBusy()){}
        }
    }

    @Config
    public static class Airplane{
        public static Servo airplane;
        public static double HOLD = .7;
        public static double LAUNCH = .5;
        public static void init(HardwareMap hardwareMap){
            airplane = hardwareMap.get(Servo.class, "airplane");
            airplane.setPosition(HOLD);
        }

        public static void launchPlane(){
            airplane.setPosition(LAUNCH);
        }
    }

    public static class LimitSwitch{
        public static DigitalChannel rightSwitch;
        public static DigitalChannel leftSwitch;
        public static void init(HardwareMap hardwareMap){
            rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");
            leftSwitch = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        }

        public static boolean getRight(){
            return rightSwitch.getState();
        }

        public static boolean getLeft(){
            return leftSwitch.getState();
        }
    }

    @Config
    public static class Color{
        public static ColorSensor color;
        static float[] hsvValues = {0F, 0F, 0F};
        final float values[] = hsvValues;
        static final double SCALE_FACTOR = 255;

        public static void init(HardwareMap hardwareMap){
            color = hardwareMap.get(ColorSensor.class, "color");
        }

        public static boolean isWhite(double threshhold){
            android.graphics.Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                    (int) (color.green() * SCALE_FACTOR),
                    (int) (color.blue() * SCALE_FACTOR),
                    hsvValues);

            return hsvValues[2] > threshhold;
        }
    }

    public static class Heading{
        private static BHI260IMU imu;
        static IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        public static void init(HardwareMap hardwareMap){
            imu = hardwareMap.get(BHI260IMU.class, "imu");
            imu.initialize(params);
            imu.resetYaw();
        }
        public static double getYaw(){
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI;
        }
        public static void reboot(){
            imu.initialize(params);
        }
        public static void reset(){
            imu.resetYaw();
        }
    }

    public static class Distance{
        public static DistanceSensor distance;
        public static void init(HardwareMap hardwareMap){
            distance = hardwareMap.get(DistanceSensor.class, "distance");
        }

        public static double getDist(DistanceUnit unit){
            return distance.getDistance(unit);
        }
    }
}
//Token: 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