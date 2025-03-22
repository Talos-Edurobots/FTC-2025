package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private Distance pos;
    private DcMotor arm;
    Arm(DcMotor arm) {
        this.arm = arm;
    }
}
