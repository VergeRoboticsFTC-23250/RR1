package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.util.Robot.Airplane;
import static org.firstinspires.ftc.teamcode.util.Robot.Chassis;
import static org.firstinspires.ftc.teamcode.util.Robot.Claw;
import static org.firstinspires.ftc.teamcode.util.Robot.Hook;
import static org.firstinspires.ftc.teamcode.util.Robot.IntakeToRest;
import static org.firstinspires.ftc.teamcode.util.Robot.LimitSwitch;
import static org.firstinspires.ftc.teamcode.util.Robot.Nicker;
import static org.firstinspires.ftc.teamcode.util.Robot.OuttakeToRest;
import static org.firstinspires.ftc.teamcode.util.Robot.RestToIntake;
import static org.firstinspires.ftc.teamcode.util.Robot.RestToOuttake;
import static org.firstinspires.ftc.teamcode.util.Robot.RobotState;
import static org.firstinspires.ftc.teamcode.util.Robot.Slides;
import static org.firstinspires.ftc.teamcode.util.Robot.robotState;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
public class DriverMode extends LinearOpMode {
    DriveThread driveThread = new DriveThread();
    TelemetryThread telemetryThread = new TelemetryThread();
    HookThread hookThread = new HookThread();
    LimitSwitchThread limitSwitchThread = new LimitSwitchThread();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);

        waitForStart();

        IntakeToRest();

        driveThread.start();
        telemetryThread.start();
        //limitSwitchThread.start();
        hookThread.start();

        while (!isStopRequested()) {

            if(!gamepad2.cross && robotState != RobotState.INTAKE && gamepad2.square){
                if(robotState == RobotState.OUTTAKE){
                    OuttakeToRest();
                    while (gamepad2.square){}
                }else{
                    RestToOuttake();
                    while (gamepad2.square){}
                }
            }else {
                if(robotState != RobotState.OUTTAKE){
                    if(gamepad2.cross && robotState != RobotState.INTAKE){
                        RestToIntake();
                    }else if(!gamepad2.cross && robotState == RobotState.INTAKE) {
                        IntakeToRest();
                    }
                }
            }

            if(robotState == RobotState.OUTTAKE && !gamepad2.square && !gamepad2.cross){
                if(gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0){
                    Slides.run(0);
                }else if(gamepad2.right_trigger > 0){
                    Slides.run(gamepad2.right_trigger);
                }else if(gamepad2.left_trigger > 0){
                    Slides.run(-gamepad2.left_trigger);
                }else{
                    Slides.run(0);
                }
            }

            Claw.setRightGrip(gamepad2.right_bumper);
            Claw.setLeftGrip(gamepad2.left_bumper);

            if(gamepad2.right_trigger > 0 && robotState == RobotState.INTAKE){
                Nicker.setRightHome();
            }else{
                if(robotState == RobotState.INTAKE){
                    Nicker.setRightOut();
                }else{
                    Nicker.setRightRest();
                }
            }

            if(gamepad2.left_trigger > 0 && robotState == RobotState.INTAKE){
                Nicker.setLeftHome();
            }else{
                if(robotState == RobotState.INTAKE){
                    Nicker.setLeftOut();
                }else{
                    Nicker.setLeftRest();
                }
            }

            if(gamepad1.dpad_up && gamepad1.triangle){
                Airplane.launchPlane();
            }
        }
    }

    class DriveThread extends Thread {
        public void run(){
            try{
                while (!isStopRequested()){
                    if(gamepad1.right_bumper){
                        Chassis.run(-gamepad1.right_stick_y * Robot.SLOW_SPEED, -gamepad1.right_stick_x * Robot.SLOW_SPEED, -gamepad1.left_stick_x * Robot.SLOW_SPEED);
                    }else{
                        Chassis.run(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
                    }
                }
            }catch (Exception e){

            }
        }
    }

    class TelemetryThread extends Thread {
        public void run(){
            try {
                while (!isStopRequested()){
                    telemetry.addData("Slide Position", Slides.getPos());
                    telemetry.addData("Slides isBusy", Slides.isBusy());
                    telemetry.addData("Slide Position Differance", Slides.getPosDifferance());
                    telemetry.addData("Slide Position individual", Slides.getPosIndividual());
                    telemetry.addData("Slide Target Pos individual", Slides.getTargetPositions());
                    telemetry.addData("Limits", LimitSwitch.getLeft() + " | " + LimitSwitch.getRight());
                    telemetry.addData("RobotState", robotState);
                    telemetry.update();
                }

            }catch (Exception e){

            }
        }
    }

    class HookThread extends Thread {
        public void run(){
            try{
                while (!isStopRequested()){
                    if(gamepad1.dpad_left && gamepad1.square){
                        while (gamepad1.dpad_left || gamepad1.square){}
                        Hook.toggle(1);
                    }
                }
            }catch (Exception e){

            }
        }
    }

    class LimitSwitchThread extends Thread {
        public void run(){
            long lastTime = System.currentTimeMillis();
            int lastCount = 0;
            try {
                while (!isStopRequested()){
                    long currentTime = System.currentTimeMillis();
                    if(LimitSwitch.getRight() && LimitSwitch.getLeft()){
                        if(currentTime - lastTime > 1000 || lastCount != 2) {
                            gamepad2.setLedColor(0, 1, 0, 1000);
                            gamepad1.setLedColor(0, 1, 0, 1000);
                            gamepad2.rumbleBlips(2);
                            lastTime = currentTime;
                        }
                        lastCount = 2;
                    }else if(LimitSwitch.getLeft() || LimitSwitch.getRight()){
                        if(currentTime - lastTime > 1000 || lastCount != 1) {
                            gamepad2.setLedColor(1, 0, 1, 1000);
                            gamepad1.setLedColor(1, 0, 1, 1000);
                            gamepad1.rumbleBlips(1);
                        }
                        lastCount = 1;
                    }else{
                        lastCount = 0;
                    }
                }
            }catch (Exception e){

            }
        }
    }
}