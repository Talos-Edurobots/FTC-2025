package org.firstinspires.ftc.teamcode.bootcamp.day3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Actuator {
    DcMotor motor;
    private Telemetry tel;
    private final int FOLDED_POS = 0;
    private final int UNFOLDED_POS = 72;
    public Actuator(Telemetry tel, HardwareMap hwmap, String leftName, String rightName) {
        this.tel = tel;
        motor =  hwmap.dcMotor.get(leftName);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(boolean up, boolean down, boolean upPos, boolean downPos) {
        if (up) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1);
        }
        else if (down) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-1);
        }
        else {
            motor.setPower(0);
        }
        if (upPos) {
            motor.setTargetPosition(FOLDED_POS);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (downPos) {
            motor.setTargetPosition(UNFOLDED_POS);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        tel.addData("actuator power", motor.getPower());
        tel.addData("actuator current",((DcMotorEx) motor).getCurrent(CurrentUnit.AMPS));
    }
}
