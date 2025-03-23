package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test", group = "Robot")
public class Test extends LinearOpMode {
    private DcMotor motor;
    private double power;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            power = gamepad1.right_trigger;
            motor.setPower(power);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
