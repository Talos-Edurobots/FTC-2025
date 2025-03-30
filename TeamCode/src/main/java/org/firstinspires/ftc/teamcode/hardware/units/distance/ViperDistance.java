package org.firstinspires.ftc.teamcode.hardware.units.distance;

import org.firstinspires.ftc.teamcode.hardware.Constants;

public class ViperDistance {
    private double value;
    private ViperUnit unit;
    ViperDistance(double value, ViperUnit unit) {
        this.value = value;
        this.unit = unit;
    }
    public static double viperTicksToMM(int ticks) {
        return (double) ticks / Constants.VIPER_TICKS_PER_MM;
    }
    public static int viperMMToTicks(double mm) {
        return (int) (Constants.VIPER_TICKS_PER_MM * mm);
    }
    public double mm() {
        if (unit == ViperUnit.TICKS) {
            return viperTicksToMM((int) value);
        }
        return value;
    }
    public int ticks() {
        if (unit == ViperUnit.MM) {
            return viperMMToTicks(value);
        }
        return (int) value;
    }

    public enum ViperUnit {
        MM,
        TICKS
    }
}

