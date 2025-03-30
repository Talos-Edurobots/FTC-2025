package org.firstinspires.ftc.teamcode.hardware.strafer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Constants;

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


    static class Motor {
        private DcMotor motor;
        private double acceleration;
        private double friction;
        private double velocity;
        private double power;
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
        public void setPower(double power) {
            this.power = power;
            motor.setPower(power);
        }
    }
}

