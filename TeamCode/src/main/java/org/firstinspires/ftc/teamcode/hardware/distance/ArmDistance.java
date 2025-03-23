package org.firstinspires.ftc.teamcode.hardware.distance;

import org.firstinspires.ftc.teamcode.hardware.Constants;

public class ArmDistance {
    private double value;
    private ArmUnit unit;
    ArmDistance(double value, ArmUnit unit) {
        this.value = value;
        this.unit = unit;
    }
    public static int armDegreesToTicks(double degrees) {
        return (int) (Constants.ARM_TICKS_PER_DEGREE * degrees);
    }
    public static double armTicksToDegrees(int ticks) {
        return (double) Constants.ARM_TICKS_PER_DEGREE / ticks;
    }
    public double degrees() {
        if (unit == ArmUnit.TICKS) {
            return armTicksToDegrees((int) value);
        }
        return value;
    }
    public int ticks() {
        if (unit == ArmUnit.DEGREES) {
            return armDegreesToTicks(value);
        }
        return (int) value;
    }
}
