package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.units.velocity.StraferMotorVelocity;

public class Strafer {
    private IMU imu;
    Motor leftFront, leftBack, rightFront, rightBack;
    Strafer(HardwareMap hwMap, boolean mockingHardware, IMU imu) {
        this.imu   = imu;
        leftFront  = new Motor(hwMap, Constants.LEFT_FRONT_MOTOR_CONFIGURATION,  DcMotorSimple.Direction.REVERSE, mockingHardware);
        leftBack   = new Motor(hwMap, Constants.LEFT_BACK_MOTOR_CONFIGURATION,   DcMotorSimple.Direction.REVERSE, mockingHardware);
        rightFront = new Motor(hwMap, Constants.RIGHT_FRONT_MOTOR_CONFIGURATION, DcMotorSimple.Direction.FORWARD, mockingHardware);
        rightBack  = new Motor(hwMap, Constants.RIGHT_BACK_MOTOR_CONFIGURATION,  DcMotorSimple.Direction.FORWARD, mockingHardware);
    }
    class Motor {
        private DcMotor motor;
        private double acceleration = 0.1; // m/s²?
        private double friction; // s¯¹
        private StraferMotorVelocity velocity;
        private boolean mockingHardware;
        Motor(HardwareMap hwMap, String name, DcMotorSimple.Direction direction, boolean mockingHardware) {
            this.mockingHardware = mockingHardware;
            if (mockingHardware) {
                motor = hwMap.dcMotor.get(name);
                motor.setDirection(direction);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        public void setPower(StraferMotorVelocity velocity) {
            this.velocity = velocity;
            motor.setPower(velocity.getP());
        }

    }
    public void fieldCentricStrafing(double x, double y, double rx) {

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        // press options button to reset IMU

        // get the orientation of the robot
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // we multiply denominator variable by a variable named "straferSpeedFactor" with a value greater than 1
        // in order to reduce the strafing speed. The normal strafing speed is to high and thus difficult to control the robot
        double denominator    =  Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        leftFront.  setPower(new StraferMotorVelocity(frontLeftPower,  StraferMotorVelocity.Units.POWER));
        leftBack.   setPower(new StraferMotorVelocity(backLeftPower,   StraferMotorVelocity.Units.POWER));
        rightFront. setPower(new StraferMotorVelocity(frontRightPower, StraferMotorVelocity.Units.POWER));
        rightBack.  setPower(new StraferMotorVelocity(backRightPower,  StraferMotorVelocity.Units.POWER));
    }
}

