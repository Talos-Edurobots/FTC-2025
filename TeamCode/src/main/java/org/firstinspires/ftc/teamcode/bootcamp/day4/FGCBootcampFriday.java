package org.firstinspires.ftc.teamcode.bootcamp.day4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "FridayBootcamp", group = "Challenges")
public class FGCBootcampFriday extends LinearOpMode {
    private DcMotor leftDrive, rightDrive, ropeDrive, intakeDrive;
    private Servo servoDrive;
    private double normalSpeed = .8; // the speed that the robot will move
    private double slowSpeed = .4;
    private double open_pos = 0;
    private double closed_pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        ropeDrive = hardwareMap.dcMotor.get("ropeDrive");
        intakeDrive = hardwareMap.dcMotor.get("intakeDrive");
        servoDrive = hardwareMap.servo.get("servoDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ropeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ropeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servoDrive.setPosition(open_pos);
        waitForStart();
        while (opModeIsActive()) {
            double forwardSpeed = -gamepad1.left_stick_y;
            double turnSpeed = gamepad1.right_stick_x;
            double decelerationAmount = gamepad1.left_trigger;
            double targetStrafingSpeed = normalSpeed - (decelerationAmount * (normalSpeed - slowSpeed));

            //drive
            rightDrive.setPower(targetStrafingSpeed * (forwardSpeed - turnSpeed));
            leftDrive.setPower(targetStrafingSpeed * (forwardSpeed + turnSpeed));
            //servo
            if (gamepad1.dpad_up){
                servoDrive.setPosition(open_pos);
            }
            else if (gamepad1.dpad_down){
                servoDrive.setPosition(closed_pos);
            }

            //intake
            if (gamepad1.left_bumper){
                intakeDrive.setPower(1);
            }
            else if (gamepad1.right_bumper) {
                intakeDrive.setPower(-1);
            }
            else {
                intakeDrive.setPower(0);
            }

            //rope
            if (gamepad1.y){
                ropeDrive.setPower(1);
            } else if (gamepad1.a) {
                ropeDrive.setPower(-1);
            }
            else {
                ropeDrive.setPower(0);
            }

            //telemetry
            telemetry.addData("target strafing speed", targetStrafingSpeed);
            telemetry.addData("left drive power", leftDrive.getPower());
            telemetry.addData("right drive power", rightDrive.getPower());
            telemetry.addData("intake current", ((DcMotorEx) intakeDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rope current", ((DcMotorEx) intakeDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}