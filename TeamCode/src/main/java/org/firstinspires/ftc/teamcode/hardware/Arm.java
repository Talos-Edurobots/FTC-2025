package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private int posInTicks;
    private DcMotor arm;
    Arm(DcMotor arm) {
        this.arm = arm;
    }
    public static int armDegreesToTicks(double degrees) {
        return (int) (Constants.ARM_TICKS_PER_DEGREE * degrees);
    }
    public static double armTicksToDegrees(int ticks) {
        return (double) Constants.ARM_TICKS_PER_DEGREE / ticks;
    }
    public void setArmPosition(int ticks) {
        posInTicks = ticks;
    }
    public void setArmPosition(double degrees) {
        posInTicks = armDegreesToTicks(degrees);
    }
    public int getArmPositionTicks() {
        return posInTicks;
    }
    public double getArmPositionDegrees() {
        return armTicksToDegrees(posInTicks);
    }
}
