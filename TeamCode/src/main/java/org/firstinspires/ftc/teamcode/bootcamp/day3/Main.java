package org.firstinspires.ftc.teamcode.bootcamp.day3;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bootcamp.DriveTrain;

@TeleOp(name = "Bootcamp Day 3", group = "Robot")
@Disabled
public class Main extends LinearOpMode {
    ElapsedTime timer;
    private DriveTrain driveTrain;
    private Actuator actuator;
    private DcMotor accelerator;
    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(telemetry, hardwareMap, "left_drive", "right_drive");
        actuator = new Actuator(telemetry, hardwareMap, "arm_drive", "right_actuator");
        accelerator = hardwareMap.get(DcMotor.class, "accelerator_drive");
        timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            timer.reset();
            if (gamepad1.y) {
                accelerator.setPower(1);
            }
            else {
                accelerator.setPower(0);
            }
            driveTrain.update(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger, timer.milliseconds()/1000);
            actuator.update(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.a, gamepad1.x);
            telemetry.update();
        }
    }
}
