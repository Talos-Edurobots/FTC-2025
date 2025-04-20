package org.firstinspires.ftc.teamcode.thesaloniki_2025;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


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
@Autonomous(name ="encoder")
//@Disabled
public class ObservationAutoThes2025 extends LinearOpMode {
    public DcMotor driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight, viper, arm;
    public boolean isRed = true, toObservation = false, recentAreaChange, recentIsRedChange;
    Servo wrist, intake;
    private int leftFrontPos, rightFrontPos, leftRearPos, rightRearPos;
    private static final double TICKS_PER_CM = 2000 // ticks
            /120.5; // cm
    private static final double TICKS_PER_DEGREE = 1615 // ticks
            /180.0; // degrees
    private static final double TICKS_PER_VIPER_MM = (537.7 * 5.8) // total ticks
            / 696; // viper slide unfolded length
    private static final double TICKS_PER_ARM_DEGREE = 28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1/360.0; // we want ticks per degree, not per rotation

    @Override
    public void runOpMode() {
        driveFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        driveFrontRight = hardwareMap.get(DcMotor.class, "right_front");
        driveRearLeft = hardwareMap.get(DcMotor.class, "left_back");
        driveRearRight = hardwareMap.get(DcMotor.class, "right_back");
        viper = hardwareMap.get(DcMotor.class, "viper_motor");
        arm = hardwareMap.get(DcMotor.class, "dc_arm");
        intake = hardwareMap.get(Servo.class, "intake_servo");
        wrist  = hardwareMap.get(Servo.class, "wrist_servo");

        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setDirection(DcMotorSimple.Direction.REVERSE);
        viper.setTargetPosition(0);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(0);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!(opModeIsActive())) {
            viper.setTargetPosition(50);
            ((DcMotorEx) viper).setVelocity(300);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("viper Current:", ((DcMotorEx) viper).getCurrent(CurrentUnit.AMPS));
            sleep(100);
            intake.setPosition(1);
            wrist.setPosition(0);
            if (gamepad1.dpad_left) {
                if (!recentAreaChange) {
                    toObservation = !toObservation;
                    recentAreaChange = true;
                }
            } else {
                recentAreaChange = false;
            }

            telemetry.addData("Position (left)", "%s", toObservation ? "Observation" : "Basket");
            telemetry.update();
        }
        waitForStart();
        if (toObservation) {
            intake.setPosition(1);
            sleep(1000);
            runArm(5.0);
            powerAllMotors(52.0, .5);
            strafe(-15.0, .5);
            runArm(80.0);
            powerAllMotors(20.5, .1);
//            runArm(60.0);
//            intake.setPosition(0);
//            powerAllMotors(-70.0, .5);
//            runArm(5.0);
//            strafe(95.0, .5);
//            powerAllMotors(100.0, .5);
//            rotate(90.0, .3);
//            powerAllMotors(-42.5, .2);
        }
        else {
            intake.setPosition(1);
            runArm(5.0);
            powerAllMotors(78.0, .5);
            rotate(-135.0, .3);
            powerAllMotors(70.0, .5);
            runArm(110.0);
            powerAllMotors(22.0, .2);
            runViper(500);
            intake.setPosition(0);
            runViper(50);
            powerAllMotors(-20.0, .5);
        }

        while (opModeIsActive()) {
            telemetry();
        }
    }
    private boolean motorsBusy() {
        return
                driveRearRight.getTargetPosition()-5 <= driveRearRight.getCurrentPosition() &&
                        driveRearRight.getTargetPosition()+5 >= driveRearRight.getCurrentPosition();
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
    private void runArm(double degrees) {
        int ticks = (int) (degrees * TICKS_PER_ARM_DEGREE);
        arm.setTargetPosition(ticks);
        arm.setPower(.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy() && opModeIsActive()) {
            telemetry();
        }
    }
    private void runViper(double mm) {
        int ticks = (int) (mm * TICKS_PER_VIPER_MM);
        viper.setTargetPosition(ticks);
        viper.setPower(.5);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viper.isBusy() && opModeIsActive()) {
            telemetry();
        }
//        arm.setPower(0);

        Log.d("myTag", String.valueOf(viper.getTargetPosition()));
    }
    private void rotate(int ticks, double speed) {
        setTargetPosition(ticks, ticks, -ticks, -ticks);
        setPower(speed);
        runMotors();
    }
    private void rotate(double degress, double speed){
        int ticks = (int) (degress * TICKS_PER_DEGREE);
        setTargetPosition(ticks, ticks, -ticks, -ticks);
        setPower(speed);
        runMotors();
    }
    private void reset() {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFrontLeft.setTargetPosition(0);
        driveFrontRight.setTargetPosition(0);
        driveRearLeft.setTargetPosition(0);
        driveRearRight.setTargetPosition(0);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void setTargetPosition(int leftFront, int leftBack, int rightFront, int rightBack){
        driveFrontLeft.setTargetPosition(leftFront);
        driveFrontRight.setTargetPosition(rightFront);
        driveRearLeft.setTargetPosition(leftBack);
        driveRearRight.setTargetPosition(rightBack);
    }
    private void setTargetPosition(int target) {
        driveFrontLeft.setTargetPosition(target);
        driveRearLeft.setTargetPosition(target);
        driveFrontRight.setTargetPosition(target);
        driveRearRight.setTargetPosition(target);
    }
    private void setPower(double power) {
        driveFrontLeft.setPower(power);
        driveRearLeft.setPower(power);
        driveFrontRight.setPower(power);
        driveRearRight.setPower(power);
    }
    private void runMotors() {
//        while (motorsBusy() && opModeIsActive()){
        do {
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry();
        }
        while (driveRearRight.isBusy() && opModeIsActive());
        sleep(100);
        reset();
    }
    private void powerAllMotors(int ticks, double speed){
        setTargetPosition(ticks);
        setPower(speed);
        runMotors();
    }
    private void powerAllMotors(double cm, double speed){
        int ticks = (int) (cm * TICKS_PER_CM);
        setTargetPosition(ticks);
        setPower(speed);
        runMotors();
    }
    private void strafe(int ticks, double speed){
        setTargetPosition(ticks, -ticks, -ticks, ticks);
        setPower(speed);
        runMotors();
    }
    private void strafe(double cm, double speed) {
        int ticks = (int) (cm * TICKS_PER_CM);
        setTargetPosition(ticks, -ticks, -ticks, ticks);
        setPower(speed);
        runMotors();
    }
    private void telemetry() {
        telemetry.addData("viper Current:", ((DcMotorEx) viper).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("arm Current:", ((DcMotorEx) arm).getCurrent(CurrentUnit.AMPS));

        telemetry.addData("viper current position", viper.getCurrentPosition());
        telemetry.addData("arm current position", arm.getCurrentPosition());

        telemetry.addData("front left current position", driveFrontLeft.getCurrentPosition());
        telemetry.addData("back left current position", driveRearLeft.getCurrentPosition());
        telemetry.addData("back right current position", driveRearRight.getCurrentPosition());
        telemetry.addData("back left current position", driveRearLeft.getCurrentPosition());

        telemetry.addData("front left target position", driveFrontLeft.getTargetPosition());
        telemetry.addData("back left target position", driveRearLeft.getTargetPosition());
        telemetry.addData("back right target position", driveRearRight.getTargetPosition());
        telemetry.addData("front right target position", driveFrontRight.getTargetPosition());
        telemetry.update();
    }

}
