package org.firstinspires.ftc.teamcode.bootcamp.final_day;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/*
 * This is the code for the autonomous period of our robot this year. We use encoders to drive, strafe,
 * rotate, to move the arm and the viper slide. We create some constants in order to to convert the
 * centimeters of the robot drive and strafe to ticks (TICKS_PER_CM), the angle that the robot rotates
 *  in degrees to ticks (TICKS_PER_DEGREE), the angle that the arm points in degrees to ticks
 * (TICKS_PER_ARM_DEGREE), and the amount that the viper slide extends in millimeters to ticks
 * (TICKS_PER_VIPER_MM).
 *
 * There are three main functions;
 *
 */
@Autonomous(name ="autonomous final")
//@Disabled
public class AutoFinal extends LinearOpMode {
    public DcMotor leftDrive, rightDrive, arm;
    private int leftPos, rightPos;
    private static final double TICKS_PER_MM = (28 * 4 * 5) / (90 * Math.PI);
    private static final double TICKS_PER_DEGREE = 1615 // ticks
            /180.0; // degrees
    private static final double TICKS_PER_VIPER_MM = (537.7 * 5.8) // total ticks
            / 696; // viper slide unfolded length
    private static final double TICKS_PER_ARM_DEGREE = 18.6;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm = hardwareMap.get(DcMotor.class, "arm");
//        driveRearLeft = hardwareMap.get(DcMotor.class, "left_back");
//        driveRearRight = hardwareMap.get(DcMotor.class, "right_back");
//        viper = hardwareMap.get(DcMotor.class, "viper_motor");
//        arm = hardwareMap.get(DcMotor.class, "dc_arm");
//        intake = hardwareMap.get(Servo.class, "intake_servo");
//        wrist  = hardwareMap.get(Servo.class, "wrist_servo");

//        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        viper.setDirection(DcMotorSimple.Direction.REVERSE);
//        viper.setTargetPosition(0);
//        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        driveRearLeft.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(0);
//        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!(opModeIsActive())) {
//            viper.setTargetPosition(50);
//            ((DcMotorEx) viper).setVelocity(300);
//            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            telemetry.addData("viper Current:", ((DcMotorEx) viper).getCurrent(CurrentUnit.AMPS));
//            sleep(100);
//            intake.setPosition(1);
//            wrist.setPosition(0);
//            if (gamepad1.dpad_left) {
//                if (!recentAreaChange) {
//                    toObservation = !toObservation;
//                    recentAreaChange = true;
//                }
//            } else {
//                recentAreaChange = false;
//            }
//
//            telemetry.addData("Position (left)", "%s", toObservation ? "Observation" : "Basket");
//            telemetry.update();
        }
        waitForStart();
//        if (toObservation) {
//            intake.setPosition(1);
//            sleep(1000);
//            runArm(5.0);
            powerAllMotors(1000.0, .5);
//            strafe(-15.0, .5);
//            runArm(80.0);
            powerAllMotors(20.0, .1);
//            runArm();
//            runArm(60.0);
//            intake.setPosition(0);
//            powerAllMotors(-70.0, .5);
//            runArm(5.0);
//            strafe(95.0, .5);
//            powerAllMotors(100.0, .5);
//            rotate(90.0, .3);
//            powerAllMotors(-42.5, .2);
//        }
//        else {
//            intake.setPosition(1);
//            runArm(5.0);
//            powerAllMotors(78.0, .5);
//            rotate(-135.0, .3);
//            powerAllMotors(70.0, .5);
//            runArm(110.0);
//            powerAllMotors(22.0, .2);
//            runViper(500);
//            intake.setPosition(0);
//            runViper(50);
//            powerAllMotors(-20.0, .5);
//        }

        while (opModeIsActive()) {
            telemetry();
        }
    }
    private void runArm(int ticks) {
        arm.setTargetPosition(ticks);
        arm.setPower(.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy() && opModeIsActive()) {
            telemetry.addData("armTargetPos", arm.getTargetPosition());
            telemetry.addData("armCurrentPos", arm.getCurrentPosition());
            telemetry.update();
        }
    }
//    private void runArm(double degrees) {
//        int ticks = (int) (degrees * TICKS_PER_ARM_DEGREE);
//        arm.setTargetPosition(ticks);
//        arm.setPower(.5);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (arm.isBusy() && opModeIsActive()) {
//            telemetry();
//        }
//    }
//    private void runViper(double mm) {
//        int ticks = (int) (mm * TICKS_PER_VIPER_MM);
//        viper.setTargetPosition(ticks);
//        viper.setPower(.5);
//        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (viper.isBusy() && opModeIsActive()) {
//            telemetry();
//        }
////        arm.setPower(0);
//
////        Log.d("myTag", String.valueOf(viper.getTargetPosition()));
//    }
    private void rotate(int ticks, double speed) {
        setTargetPosition(ticks, -ticks);
        setPower(speed);
        runMotors();
    }
    private void rotate(double degress, double speed){
        int ticks = (int) (degress * TICKS_PER_DEGREE);
        setTargetPosition(ticks, -ticks);
        setPower(speed);
        runMotors();
    }
    private void reset() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(0);
        rightDrive.setTargetPosition(0);
//        driveRearLeft.setTargetPosition(0);
//        driveRearRight.setTargetPosition(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void setTargetPosition(int left, int right){
        leftDrive.setTargetPosition(left);
        rightDrive.setTargetPosition(right);
//        driveRearLeft.setTargetPosition(leftBack);
//        driveRearRight.setTargetPosition(rightBack);
    }
    private void setTargetPosition(int target) {
        leftDrive.setTargetPosition(target);
//        driveRearLeft.setTargetPosition(target);
        rightDrive.setTargetPosition(target);
//        driveRearRight.setTargetPosition(target);
    }
    private void setPower(double power) {
        leftDrive.setPower(power);
//        driveRearLeft.setPower(power);
        rightDrive.setPower(power);
//        driveRearRight.setPower(power);
    }
    private void runMotors() {
//        while (motorsBusy() && opModeIsActive()){
        do {
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            driveRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            driveRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry();
        }
        while (leftDrive.isBusy() && opModeIsActive());
        sleep(100);
        reset();
    }
    private void powerAllMotors(int ticks, double speed){
        setTargetPosition(ticks);
        setPower(speed);
        runMotors();
    }
    private void powerAllMotors(double mm, double speed){
        int ticks = (int) (mm * TICKS_PER_MM);
        setTargetPosition(ticks);
        setPower(speed);
        runMotors();
    }
    private void strafe(int ticks, double speed){
        setTargetPosition(ticks, -ticks);
        setPower(speed);
        runMotors();
    }
    private void telemetry() {
//        telemetry.addData("viper Current:", ((DcMotorEx) viper).getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("arm Current:", ((DcMotorEx) arm).getCurrent(CurrentUnit.AMPS));
//
//        telemetry.addData("viper current position", viper.getCurrentPosition());
//        telemetry.addData("arm current position", arm.getCurrentPosition());

//        telemetry.addData("front left current position", leftDrive.getCurrentPosition());
//        telemetry.addData("back left current position", driveRearLeft.getCurrentPosition());
//        telemetry.addData("back right current position", driveRearRight.getCurrentPosition());
//        telemetry.addData("back left current position", driveRearLeft.getCurrentPosition());

        telemetry.addData("front left target position", leftDrive.getTargetPosition());
//        telemetry.addData("back left target position", driveRearLeft.getTargetPosition());
//        telemetry.addData("back right target position", driveRearRight.getTargetPosition());
        telemetry.addData("front right target position", rightDrive.getTargetPosition());
        telemetry.update();
    }

}
