package org.firstinspires.ftc.teamcode.bootcamp.day4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test", group = "Robot")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor dc1, dc2, dc3, dc4;
        dc1 = hardwareMap.get(DcMotor.class, "ropeDrive");
        dc2 = hardwareMap.get(DcMotor.class, "right_drive");
        dc3 = hardwareMap.get(DcMotor.class, "left_drive");
        dc4 = hardwareMap.get(DcMotor.class, "intakeDrive");
        dc1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dc2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dc3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dc4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.right_trigger;
            double power2 = gamepad1.left_trigger;
            double power3 = gamepad1.left_stick_y;
            double power4 = gamepad1.right_stick_x;
            dc1.setPower(power);
            dc2.setPower(power2);
            dc3.setPower(power3);
            dc4.setPower(power4);
            telemetry.addData("Power 1", power);
            telemetry.addData("Power 2", power2);
            telemetry.addData("Power 3", power3);
            telemetry.addData("Power 4", power4);
            telemetry.update();
        }
    }
}
