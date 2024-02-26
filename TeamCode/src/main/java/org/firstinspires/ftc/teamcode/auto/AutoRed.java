//package org.firstinspires.ftc.teamcode.auto;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import static org.firstinspires.ftc.teamcode.auto.AutoBlue.strafePark;
//import static org.firstinspires.ftc.teamcode.util.Robot.Chassis.drive;
//import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition;
//import static org.firstinspires.ftc.teamcode.util.Robot.AllianceSide.*;
//import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.CENTER;
//import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.LEFT;
//import static org.firstinspires.ftc.teamcode.util.Robot.PropPosition.RIGHT;
//import static org.firstinspires.ftc.teamcode.util.Robot.pGainHeading;
//
//import androidx.annotation.NonNull;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.auto.apriltag.AprilTagLocalization;
//import org.firstinspires.ftc.teamcode.auto.opencv.DetectionRed;
//import org.firstinspires.ftc.teamcode.auto.opencv.Vision;
//import org.firstinspires.ftc.teamcode.util.Robot;
//import org.tensorflow.lite.task.vision.detector.Detection;
//
//@Autonomous
//@Config
//public class AutoRed extends LinearOpMode {
//    public static int firstPixelDelay = 150;
//    public static double pushIn = 6;
//    public static double pushInPower = 0.5;
//    public static double aprilTagRunDuration = 3000;
//
//    //Drive To Back
//    public static double KpForward = 2;
//    public static double KiForward = 1;
//    public static double KdForward = 0.16;
//
//    public static double KpHeading = 2;
//    public static double KiHeading = 0;
//    public static double KdHeading = 0.001;
//
//    public static double referenceForward = 0.13;
//    public static double referenceHeading = Math.PI * 1/2;
//    public static double takeoverDist = 1;
//    public static double power = -1;
//
//    //Intake
//    public static double activePower = -0.25;
//    public static double overdrivePower = -0.5;
//    public static double standbyPower = 0;
//    public static double backPower = 0.15;
//    public static int backTime = 150;
//    public static double threshhold = 950;
//    public static double strafePower = -0.3;
//    public static int t1 = 250;
//    public static double runToBackCancelThreshold = 0.02;
//
//    public static double toCenter = 20;
//    public static double toCenterPower = -0.5;
//
//    public static double outtakeTime = 3000;
//
//    public static double forwardDist = 68;
//    public static double strafeDist = 12;
//
//    public static double strafePark = 20;
//    public static double strafe2nd = 0.5;
//
//    PropPosition position = LEFT;
//    DigitalChannel limitL;
//    DigitalChannel limitR;
//    @Config
//    public static class LeftSpikeMark{
//        public static Action action;
//        public static double heading = -90;
//
//        public static double x = 26;
//        public static double y = -4;
//
//        public static double x2 = 26;
//        public static double y2 = 6;
//
//        public static void buildAction(){
//            action = drive.actionBuilder(drive.pose)
//                    .afterTime(0.2, telemetryPacket -> {
//                        Robot.Arm.setIntake();
//                        Robot.Claw.setIntake();
//                        return false;
//                    })
//                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
//                    .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(heading))
//                    .stopAndAdd(telemetryPacket -> {
//                        Robot.Claw.setLeftGrip(true);
//                        return false;
//                    })
//                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
//                    .build();
//        }
//    }
//
//    @Config
//    public static class CenterSpikeMark{
//        public static Action action;
//        public static double heading = -90;
//
//        public static double x = 36;
//        public static double y = -14;
//
//        public static double x2 = 36;
//        public static double y2 = -4;
//
//        public static void buildAction(){
//            action = drive.actionBuilder(drive.pose)
//                    .afterTime(0.2, telemetryPacket -> {
//                        Robot.Arm.setIntake();
//                        Robot.Claw.setIntake();
//                        return false;
//                    })
//                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
//                    .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(heading))
//                    .stopAndAdd(telemetryPacket -> {
//                        Robot.Claw.setLeftGrip(true);
//                        return false;
//                    })
//                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
//                    .build();
//        }
//    }
//
//    @Config
//    public static class RightSpikeMark{
//        public static Action action;
//        public static double heading = -90;
//
//        public static double x = 26;
//        public static double y = -24;
//
//        public static double x2 = 26;
//        public static double y2 = -16;
//
//        public static void buildAction(){
//            action = drive.actionBuilder(drive.pose)
//                    .afterTime(0.2, telemetryPacket -> {
//                        Robot.Arm.setIntake();
//                        Robot.Claw.setIntake();
//                        return false;
//                    })
//                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
//                    .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(heading))
//                    .stopAndAdd(telemetryPacket -> {
//                        Robot.Claw.setLeftGrip(true);
//                        return false;
//                    })
//                    .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
//                    .build();
//        }
//    }
//
//    public void runOpMode() throws InterruptedException {
//        Robot.init(hardwareMap);
//        Vision.init(hardwareMap, RED);
//        limitL = hardwareMap.get(DigitalChannel.class, "limitR");
//        limitR = hardwareMap.get(DigitalChannel.class, "limitL");
//        LeftSpikeMark.buildAction();
//        CenterSpikeMark.buildAction();
//        RightSpikeMark.buildAction();
//
//        initLoop(); //Detection loop
//        position = Vision.getPosition();
//        waitForStart();
//        DetectionRed.webcam.stopStreaming();
//
//        switch (position){
//            case LEFT:
//                DetectionRed.webcam.closeCameraDeviceAsync(() -> {
//                    try {
//                        AprilTagLocalization.init(hardwareMap, 4);
//                    } catch (InterruptedException ignored) {}
//                });
//                strafePark += 6; //-= for red;
//                strafe2nd += 0.5;
//                Actions.runBlocking(LeftSpikeMark.action);
//                break;
//            case CENTER:
//                DetectionRed.webcam.closeCameraDeviceAsync(() -> {
//                    try {
//                        AprilTagLocalization.init(hardwareMap, 5);
//                    } catch (InterruptedException ignored) {}
//                });
//                Actions.runBlocking(CenterSpikeMark.action);
//                break;
//            case RIGHT:
//                DetectionRed.webcam.closeCameraDeviceAsync(() -> {
//                    try {
//                        AprilTagLocalization.init(hardwareMap, 6);
//                    } catch (InterruptedException ignored) {}
//                });
//                strafePark -= 6; //+= for red
//                Actions.runBlocking(RightSpikeMark.action);
//                break;
//        }
//
//        boolean isOuttake = false;
//        long startTime = System.currentTimeMillis();
//
//        Robot.Claw.setBothGrips(false);
//        Robot.Claw.setOuttake();
//        sleep(500);
//        Robot.Arm.setRest();
//
//        while (opModeIsActive() && (System.currentTimeMillis() - startTime < aprilTagRunDuration)){
//            if(System.currentTimeMillis() - startTime > firstPixelDelay && !isOuttake){
//                Robot.Arm.setOuttake();
//                Robot.Claw.setOuttake();
//                isOuttake = true;
//            }
//            AprilTagLocalization.update();
//            drive.updatePoseEstimate();
//        }
//
//        Robot.Chassis.runY(strafe2nd, 0.5);
//
//        Robot.Chassis.runX(pushIn, pushInPower);
//
//        Thread.sleep(500);
//
//        Robot.Claw.setBothGrips(true);
//
//        Thread.sleep(500);
//        Robot.Heading.reboot();
//
//        Robot.Chassis.runX(pushIn, -pushInPower, (Math.PI * 3/2));
//
//        Robot.Arm.setRest();
//        Robot.Claw.setRest();
//
//        Robot.Heading.reboot();
//
//        Robot.Chassis.runY(strafePark, power);
//
//        //Robot.Chassis.runX(10, -power);
//
//        //firstPlus2();
//        //secondPlus2();
//
//        Thread.sleep(1000);
//    }
//
//    public void initLoop(){
//        while (!isStarted() && !isStopRequested()){
//            telemetry.addData("Detected", Vision.getPosition());
//            telemetry.update();
//            position = Vision.getPosition();
//            sleep(50);
//        }
//    }
//
//    public void runToBack(){
//        ElapsedTime timerHeading = new ElapsedTime();
//
//        double integralSumHeading = 0;
//        double lastErrorHeading = 0;
//
//        double dist = Robot.Distance.getDist(DistanceUnit.METER);
//
//        while (opModeIsActive() && dist > takeoverDist){
//            double heading = Robot.Heading.getYaw();
//            double errorHeading = referenceHeading - heading;
//            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();
//            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());
//            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);
//
//            dist = Robot.Distance.getDist(DistanceUnit.METER);
//
//            telemetry.addData("dist", dist);
//            telemetry.update();
//
//            Robot.Chassis.run(power, 0, outHeading);
//
//            lastErrorHeading = errorHeading;
//            timerHeading.reset();
//        }
//
//        Robot.Nicker.setOut();
//
//        double integralSumForward = 0;
//
//        double lastErrorForward = 0;
//
//        ElapsedTime timer = new ElapsedTime();
//
//        double startTime = System.currentTimeMillis();
//
//        while (opModeIsActive() && Math.abs(referenceForward - dist) > runToBackCancelThreshold) {
//
//            if(System.currentTimeMillis() - startTime > 500 && startTime != -1){
//                Robot.Claw.setIntake();
//                startTime = -1;
//            }
//            dist = Robot.Distance.getDist(DistanceUnit.METER);
//            double heading = Robot.Heading.getYaw();
//
//            double errorForward = referenceForward - dist;
//            double errorHeading = referenceHeading - heading;
//
//            double derivativeForward = (errorForward - lastErrorForward) / timer.seconds();
//            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();
//
//            integralSumForward = integralSumForward + (errorForward * timer.seconds());
//            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());
//
//            double outForward = (KpForward * errorForward) + (KiForward * integralSumForward) + (KdForward * derivativeForward);
//            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);
//
//            Robot.Chassis.run(outForward, 0, outHeading);
//
//            lastErrorForward = errorForward;
//            lastErrorHeading = errorHeading;
//
//            timer.reset();
//            timerHeading.reset();
//
//            telemetry.addData("dist", dist);
//            telemetry.update();
//        }
//    }
//    public void runToBack2(){
//        Robot.Heading.reboot();
//        ElapsedTime timerHeading = new ElapsedTime();
//
//        double integralSumHeading = 0;
//        double lastErrorHeading = 0;
//
//        double dist = Robot.Distance.getDist(DistanceUnit.METER);
//
//        while (opModeIsActive() && dist > takeoverDist){
//            double heading = Robot.Heading.getYaw();
//            double errorHeading = referenceHeading - heading;
//            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();
//            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());
//            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);
//
//            dist = Robot.Distance.getDist(DistanceUnit.METER);
//
//            telemetry.addData("dist", dist);
//            telemetry.update();
//
//            Robot.Chassis.run(power, 0, outHeading);
//
//            lastErrorHeading = errorHeading;
//            timerHeading.reset();
//        }
//
//        Robot.Nicker.setOut();
//
//        double integralSumForward = 0;
//
//        double lastErrorForward = 0;
//
//        ElapsedTime timer = new ElapsedTime();
//
//        double startTime = System.currentTimeMillis();
//
//        while (opModeIsActive() && Math.abs(referenceForward - dist) > runToBackCancelThreshold) {
//
//            if(System.currentTimeMillis() - startTime > 500 && startTime != -1){
//                Robot.Claw.setIntake();
//                startTime = -1;
//            }
//            dist = Robot.Distance.getDist(DistanceUnit.METER);
//            double heading = Robot.Heading.getYaw();
//
//            double errorForward = referenceForward - dist;
//            double errorHeading = referenceHeading - heading;
//
//            double derivativeForward = (errorForward - lastErrorForward) / timer.seconds();
//            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();
//
//            integralSumForward = integralSumForward + (errorForward * timer.seconds());
//            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());
//
//            double outForward = (KpForward * errorForward) + (KiForward * integralSumForward) + (KdForward * derivativeForward);
//            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);
//
//            Robot.Chassis.run(outForward, 0, outHeading);
//
//            lastErrorForward = errorForward;
//            lastErrorHeading = errorHeading;
//
//            timer.reset();
//            timerHeading.reset();
//
//            telemetry.addData("dist", dist);
//            telemetry.update();
//        }
//
//        Robot.Claw.setBothGrips(true);
//
//        integralSumForward = 0;
//
//        lastErrorForward = 0;
//
//        timer = new ElapsedTime();
//
//        while (opModeIsActive() && !Robot.Color.isWhite(threshhold)) {
//            dist = Robot.Distance.getDist(DistanceUnit.METER);
//            double heading = Robot.Heading.getYaw();
//
//            double errorForward = referenceForward - dist;
//            double errorHeading = referenceHeading - heading;
//
//            double derivativeForward = (errorForward - lastErrorForward) / timer.seconds();
//            double derivativeHeading = (errorHeading - lastErrorHeading) / timerHeading.seconds();
//
//            integralSumForward = integralSumForward + (errorForward * timer.seconds());
//            integralSumHeading = integralSumHeading + (errorHeading * timerHeading.seconds());
//
//            double outForward = (KpForward * errorForward) + (KiForward * integralSumForward) + (KdForward * derivativeForward);
//            double outHeading = (KpHeading * errorHeading) + (KiHeading * integralSumHeading) + (KdHeading * derivativeHeading);
//
//            Robot.Chassis.run(outForward, -0.5, outHeading);
//
//            lastErrorForward = errorForward;
//            lastErrorHeading = errorHeading;
//
//            timer.reset();
//            timerHeading.reset();
//
//            telemetry.addData("dist", dist);
//            telemetry.update();
//        }
//    }
//
//    public void intake() throws InterruptedException {
//        Robot.Claw.setBothGrips(true);
//
//        while (opModeIsActive() && !Robot.Color.isWhite(threshhold)){
//            Robot.Chassis.run(0, strafePower, 0);
//            drive.updatePoseEstimate();
//        }
//        while (opModeIsActive()){
//            telemetry.addData("", limitL.getState() + " | " + limitR.getState());
//            drive.updatePoseEstimate();
//
//            if(!limitL.getState() && !limitR.getState()){
//                Robot.Chassis.drive.rightFront.setPower(activePower);
//                Robot.Chassis.drive.rightBack.setPower(activePower);
//                Robot.Chassis.drive.leftFront.setPower(activePower);
//                Robot.Chassis.drive.leftBack.setPower(activePower);
//            }else if(!limitL.getState() && limitR.getState()){
//                Robot.Chassis.drive.rightFront.setPower(standbyPower);
//                Robot.Chassis.drive.rightBack.setPower(standbyPower);
//                Robot.Chassis.drive.leftFront.setPower(overdrivePower);
//                Robot.Chassis.drive.leftBack.setPower(overdrivePower);
//            }else if(limitL.getState() && !limitR.getState()){
//                Robot.Chassis.drive.rightFront.setPower(overdrivePower);
//                Robot.Chassis.drive.rightBack.setPower(overdrivePower);
//                Robot.Chassis.drive.leftFront.setPower(standbyPower);
//                Robot.Chassis.drive.leftBack.setPower(standbyPower);
//            }else if(limitR.getState() && limitL.getState()){
//                break;
//            }
//            telemetry.update();
//        }
//
//        Robot.Chassis.run(backPower, 0, 0);
//        Thread.sleep(backTime);
//        drive.updatePoseEstimate();
//        Robot.Chassis.run(0, 0, 0);
//        Robot.Nicker.setHome();
//        Thread.sleep(t1 + 500);
//        Robot.Claw.setBothGrips(false);
//        Thread.sleep(t1);
//        Robot.Nicker.setOut();
//        Thread.sleep(t1);
////        Robot.Nicker.setOut();
////        Thread.sleep(t1);
////        Robot.Claw.setRest();
////        Robot.Arm.setRest();
////        Thread.sleep(regripDelay);
////        Robot.Claw.setBothGrips(true);
////        Thread.sleep(regriptime);
////        Robot.Claw.setBothGrips(false);
//    }
//    public void firstPlus2() throws InterruptedException {
//        Robot.Claw.setBothGrips(true);
//        Robot.Heading.reboot();
//        sleep(250);
//
//        Robot.Chassis.runX(pushIn, -pushInPower);
//        Robot.Claw.setRest();
//        Robot.Claw.setBothGrips(false);
//        Robot.Arm.setRest();
//        Robot.Chassis.runY(toCenter, -toCenterPower);
//        Robot.Arm.setIntake();
//        Robot.Claw.setOuttake();
//
//        runToBack();
//
//        Robot.Chassis.run(0, 0, 0);
//
//        intake();
//
//        Robot.Chassis.runX(4, -power);
//        Robot.Nicker.setOut();
//        Robot.Chassis.runY(4, -power);
//        Robot.Arm.setRest();
//        Robot.Claw.setRest();
//
//        double startTime = System.currentTimeMillis();
//
//        Robot.Heading.reboot();
//        double currentY = drive.pose.position.y;
//        int step = 0;
//        while (Math.abs(currentY - drive.pose.position.y) < forwardDist){
//            if(System.currentTimeMillis() - startTime >= 250 && step == 0){
//                Robot.Claw.setBothGrips(true);
//                step++;
//            }
//
//            if(System.currentTimeMillis() - startTime >= 750 && step == 1){
//                Robot.Claw.setBothGrips(false);
//                step++;
//            }
//
//            if(System.currentTimeMillis() - startTime >= 1000 && step == 0){
//                Robot.Claw.setOuttake();
//                step++;
//            }
//
//            Robot.Chassis.run(-power, 0, ((Math.PI * 1/2) - Robot.Heading.getYaw()) * pGainHeading);
//            drive.updatePoseEstimate();
//        }
//        Robot.Chassis.run(0, 0, 0);
//
//        Robot.Chassis.runY(strafeDist, power);
//
//        AprilTagLocalization.setDesiredTagId(-1);
//
//        Robot.Arm.setOuttake();
//
//        startTime = System.currentTimeMillis();
//
//        while (opModeIsActive() && System.currentTimeMillis() - startTime < outtakeTime){
//            AprilTagLocalization.update();
//        }
//
//        Robot.Claw.setOuttake();
//
//        Robot.Chassis.runX(pushIn + 2, pushInPower);
//
//        Robot.Claw.setBothGrips(true);
//    }
//    public void secondPlus2() throws InterruptedException {
//        Robot.Claw.setBothGrips(true);
//
//        sleep(250);
//
//        Robot.Chassis.runX(pushIn, -pushInPower);
//        Robot.Claw.setRest();
//        Robot.Claw.setBothGrips(false);
//        Robot.Arm.setRest();
//        Robot.Chassis.runY(toCenter, -toCenterPower);
//        Robot.Arm.setIntake();
//        Robot.Claw.setOuttake();
//
//        runToBack2();
//        Robot.Chassis.run(0, 0, 0);
//
//        //intake
//        Robot.Chassis.runX(1, power);
//        Robot.Nicker.setHome();
//        sleep(500);
//        Robot.Claw.setBothGrips(false);
//
//        Robot.Chassis.runX(4, -power);
//        Robot.Nicker.setOut();
//        Robot.Chassis.runY(4, -power);
//        Robot.Arm.setRest();
//        Robot.Claw.setRest();
//
//        double startTime = System.currentTimeMillis();
//
//        Robot.Heading.reboot();
//        double currentY = drive.pose.position.y;
//        int step = 0;
//        while (Math.abs(currentY - drive.pose.position.y) < 48){
//            if(System.currentTimeMillis() - startTime >= 250 && step == 0){
//                Robot.Claw.setBothGrips(true);
//                step++;
//            }
//
//            if(System.currentTimeMillis() - startTime >= 750 && step == 1){
//                Robot.Claw.setBothGrips(false);
//                step++;
//            }
//
//            if(System.currentTimeMillis() - startTime >= 1000 && step == 0){
//                Robot.Claw.setOuttake();
//                step++;
//            }
//
//            Robot.Chassis.run(-power, 0, ((Math.PI * 1/2) - Robot.Heading.getYaw()) * pGainHeading);
//            drive.updatePoseEstimate();
//        }
//        Robot.Chassis.run(0, 0, 0);
//
//        AprilTagLocalization.setDesiredTagId(-1);
//
//        Robot.Arm.setOuttake();
//
//        startTime = System.currentTimeMillis();
//
//        while (opModeIsActive() && System.currentTimeMillis() - startTime < outtakeTime){
//            AprilTagLocalization.update();
//        }
//
//        Robot.Claw.setOuttake();
//
//        Robot.Chassis.runX(pushIn + 2, pushInPower);
//
//        Robot.Claw.setBothGrips(true);
//    }
//}