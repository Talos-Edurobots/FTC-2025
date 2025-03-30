package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

class StraferMotor {
    private DcMotor motor;
    StraferMotor(HardwareMap hwMap, String name, DcMotorSimple.Direction direction) {
        motor = hwMap.dcMotor.get(name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
public class Strafer {
    private IMU imu;
    Strafer(HardwareMap hwMap, IMU imu) {
        this.imu = imu;
    }

}
