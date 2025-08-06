package org.firstinspires.ftc.teamcode.bootcamp.final_day;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="TeleOp", group="Challenges")
public class TeleOpFinal extends OpMode {

    // Module Initialization
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx left_drive;
    private DcMotorEx right_drive;
    private DcMotorEx arm;
    private Servo arm_servo;
    private Servo front_servo;

    @Override
    public void init() {
        // Basic Motor Setup
        left_drive = hardwareMap.get(DcMotorEx.class, "left_drive");
        right_drive = hardwareMap.get(DcMotorEx.class, "right_drive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        front_servo = hardwareMap.get(Servo.class, "front_servo");

        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);

        // Don't let the wheels overshoot
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status" ,"Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double left_power;
        double right_power;
        double arm_power = 0.6;
        double slow = 0.3, medium = 0.5, fast = 0.7;

        // Update motor power based on gamepad values
        // POV mode
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // Robot moves faster while sprint button is being hed down
        if (gamepad1.right_trigger > 0.5) {
            left_power = Range.clip(drive - turn, -fast, fast);
            right_power = Range.clip(drive + turn, -fast, fast);
            // And moves slower while slow button is pressed
        } else if (gamepad1.left_trigger > 0.5){
            left_power = Range.clip(drive + turn, -slow, slow);
            right_power = Range.clip(drive - turn, -slow, slow);
        } else {
            // Else moves normally
            left_power = Range.clip(drive + turn, -medium, medium);
            right_power = Range.clip(drive - turn, -medium, medium);
        }

        left_drive.setPower(left_power);
        right_drive.setPower(right_power);

        // Control the robot arm
        if (gamepad2.dpad_up) {
            arm.setPower(arm_power);
        } else if (gamepad2.dpad_down) {
            arm.setPower(-arm_power);
        } else {
            arm.setPower(0);
        }

        // Control the robot servo
        if (gamepad2.y) {
            arm_servo.setPosition(1);
        } else if (gamepad2.x) {
            arm_servo.setPosition(0);
        }

        if (gamepad2.a) {
            front_servo.setPosition(1);
        } else if (gamepad2.b) {
            front_servo.setPosition(0);
        }

        // Debug Data
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left power (%.2f), right power (%.2f), left Amps (%.2f), right Amps (%.2f)", left_drive.getPower(), right_drive.getPower(), left_drive.getCurrent(CurrentUnit.AMPS), right_drive.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor Arm", "arm power (%.2f), arm Amps (%.2f)", arm.getPower(), arm.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}

