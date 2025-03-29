package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

class StraferMotors {
    private DcMotor rf, rb, lf, lb;
    StraferMotors(HardwareMap hwMap) {
        rf = hwMap.dcMotor.get(Constants.RIGHT_FRONT_MOTOR_CONFIGURATION);
        rb = hwMap.dcMotor.get(Constants.RIGHT_BACK_MOTOR_CONFIGURATION);
        lf = hwMap.dcMotor.get(Constants.LEFT_FRONT_MOTOR_CONFIGURATION);
        lb = hwMap.dcMotor.get(Constants.LEFT_BACK_MOTOR_CONFIGURATION);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
public class Strafer {
}
