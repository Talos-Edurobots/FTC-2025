package org.firstinspires.ftc.teamcode.hardware;

public class Configuration {
    public static final int ARM_MOTOR_RPM = 223;
    public static final int TICKS = (int) (
            28 // number of encoder ticks per rotation of the bare motor
                    * (250047.0 / 4913.0) // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * (100.0 / 20.0) // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * (1/360.0) // we want ticks per degree, not per rotation
    ); // 250047.0 / 4913.0 * 100.0/20.0 * 1/360.0

}
