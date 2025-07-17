package org.firstinspires.ftc.teamcode.bootcamp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="drivetrain", group = "tests")
@Disabled
public class BasicTeleOp extends LinearOpMode {
    DcMotor leftDrive, rightDrive;
    double normalDrivingSpeed = 1.0;
    double slowDrivingSpeed = 0.5;
    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class,    "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set the left motor to reverse direction
        rightDrive.setDirection(DcMotor.Direction.FORWARD); // Set the right motor to forward direction
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.left_stick_y;
            double deceleration_amount = gamepad1.left_trigger;
            double strafingSpeed = normalDrivingSpeed - (deceleration_amount * (normalDrivingSpeed - slowDrivingSpeed));
            rightDrive.setPower(strafingSpeed * (x+y));
            leftDrive.setPower(strafingSpeed * (x-y));
            telemetry.addData("left power", leftDrive.getPower());
            telemetry.addData("right power", rightDrive.getPower());
            telemetry.addData("strafing speed", strafingSpeed);
            telemetry.update();
        }
    }
}
