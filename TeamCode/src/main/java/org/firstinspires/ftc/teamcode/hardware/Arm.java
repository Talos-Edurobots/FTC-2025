package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.units.distance.ArmDistance;

public class Arm {
    private ArmDistance pos;
    private DcMotor arm;
    Arm() {
        arm = hardwareMap.get(DcMotor.class, Constants.ARM_MOTOR_CONFIGURATION); //the arm motor
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) arm).setCurrentAlert(5, CurrentUnit.AMPS);
        arm.setTargetPosition(0);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPosition(ArmDistance position) {
        pos = position;
    }
    public ArmDistance getPosition() {
        return pos;
    }
    public void runArm() {
        arm.setTargetPosition(pos.ticks());
        ((DcMotorEx) arm).setVelocity(Constants.ARM_VELOCITY);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
    }
}
