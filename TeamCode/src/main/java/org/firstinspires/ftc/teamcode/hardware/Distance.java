package org.firstinspires.ftc.teamcode.hardware;

public class Distance {
    private Unit unit;
    private double value;
    Distance(Unit unit, double value){
        this.unit = unit;
        this.value = value;
    }
    public static double robotTicksPerMm(int ticks) {
        return (double) Constants.ROBOT_TICKS_PER_MM / ticks;
    }
    public static int robotMmToTicks(double mm) {
        return (int) (Constants.ROBOT_TICKS_PER_MM * mm);
    }
    public static int armDegreesToTicks(double degrees) {
        return (int) (Constants.ARM_TICKS_PER_DEGREE * degrees);
    }
    public static double armTicksToDegrees(int ticks) {
        return (double) Constants.ARM_TICKS_PER_DEGREE / ticks;
    }
    public static double viperTicksToMM(int ticks) {
        return (double) Constants.VIPER_TICKS_PER_MM / ticks;
    }
    public static int viperMMToTicks(double mm) {
        return (int) (Constants.VIPER_TICKS_PER_MM * mm);
    }
    public double armDegrees() {
        if (unit == Unit.ARM_TICKS) {
            return armTicksToDegrees((int) value);
        }
        else if (unit == Unit.ARM_DEGREES){
            return value;
        }
        else {
            throw new IllegalArgumentException();
        }
    }
    public int armTicks() {
        if (unit == Unit.ARM_TICKS) {
            return (int) value;
        }
        else if (unit == Unit.ARM_DEGREES) {
            return armDegreesToTicks(value);
        }
        else {
            throw new IllegalArgumentException();
        }
    }
    public double viperMm() {
        if (unit == Unit.VIPER_TICKS) {
            return viperTicksToMM((int) value);
        }
        else if (unit == Unit.VIPER_MM){
            return value;
        }
        else {
            throw new IllegalArgumentException();
        }
    }
    public int viperTicks() {
        if (unit == Unit.VIPER_TICKS) {
            return (int) value;
        }
        else if (unit == Unit.VIPER_MM) {
            return viperMMToTicks(value);
        }
        else {
            throw new IllegalArgumentException();
        }
    }
    public double robotMm() {
        if (unit == Unit.ROBOT_MM) {
            return value;
        }
        else if (unit == Unit.ROBOT_TICKS) {
            return robotTicksPerMm((int) value);
        }
        else {
            throw new IllegalArgumentException();
        }
    }
}
