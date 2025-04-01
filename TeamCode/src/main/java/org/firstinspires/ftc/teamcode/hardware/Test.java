package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Test extends LinearOpMode {
    Strafer strafer;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        strafer = new Strafer(hardwareMap, false, imu);
        waitForStart();
        while (opModeIsActive()) {
            strafer.fieldCentricStrafing(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
