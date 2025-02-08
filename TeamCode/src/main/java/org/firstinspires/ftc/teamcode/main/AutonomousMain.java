package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="AutonomousMain", group="Robot")
public class AutonomousMain extends LinearOpMode {
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public IMU imu;
    public SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pos;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        imu = hardwareMap.get(IMU.class, "imu");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        // make sure we can stop execution during initialization
        imu.initialize(parameters);
        waitForStart();
        if (isStopRequested()){
            return;
        }
        while (opModeIsActive()) {
            pos = otos.getPosition();
            leftFront.setPower(1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(1);
            Thread.sleep(500);
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            Thread.sleep(500);
        }
    }

    public void gotoPosition(SparkFunOTOS.Pose2D targetPosition) {
        SparkFunOTOS.Pose2D currentPosition = otos.getPosition();
        double dx = targetPosition.x - currentPosition.x;
        double dy = targetPosition.y - currentPosition.y;

//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
        double rx = 0;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = dx * Math.cos(-botHeading) - dy * Math.sin(-botHeading);
        double rotY = dx * Math.sin(-botHeading) + dy * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
}
