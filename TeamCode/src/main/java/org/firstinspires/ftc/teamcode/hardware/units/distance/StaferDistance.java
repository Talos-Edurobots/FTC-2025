package org.firstinspires.ftc.teamcode.hardware.units.distance;

import org.firstinspires.ftc.teamcode.hardware.Constants;

public class StaferDistance {
    private double value;
    private StaferUnit unit;
    StaferDistance(double value, StaferUnit unit) {
        this.value = value;
        this.unit = unit;
    }
    public static double robotTicksToMm(int ticks) {
        return (double) ticks / Constants.ROBOT_TICKS_PER_MM;
    }
    public static int robotMmToTicks(double mm) {
        return (int) (Constants.ROBOT_TICKS_PER_MM * mm);
    }
    public double mm() {
        if (unit == StaferUnit.TICKS) {
            return robotTicksToMm((int) value);
        }
        return value;
    }
    public int ticks() {
        if (unit == StaferUnit.MM) {
            return robotMmToTicks(value);
        }
        return (int) value;
    }

    public enum StaferUnit {
        MM,
        TICKS
    }
}
