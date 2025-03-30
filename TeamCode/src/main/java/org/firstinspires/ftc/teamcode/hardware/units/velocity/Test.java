package org.firstinspires.ftc.teamcode.hardware.units.velocity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@TeleOp(name = "Testing velocity", group = "robot")
@Disabled
public class Test extends LinearOpMode {
    StraferMotorVelocity velocity = new StraferMotorVelocity(1.56, StraferMotorVelocity.Units.METERS_PER_SECOND);
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("r/m", velocity.getRpm());
            telemetry.addData("p", velocity.getP());
            telemetry.addData("m/s", velocity.getMps());
            telemetry.update();
        }
    }
}
