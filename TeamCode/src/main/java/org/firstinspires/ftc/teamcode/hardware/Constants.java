package org.firstinspires.ftc.teamcode.hardware;

public class Constants {
    public static final int ARM_TICKS_PER_DEGREE = (int) (
            28 // number of encoder ticks per rotation of the bare motor
                    * (250047.0 / 4913.0) // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * (100.0 / 20.0) // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * (1/360.0) // we want ticks per degree, not per rotation
    );
    public static final int VIPER_TICKS_PER_MM = (int) (
        (
            (
                537.7 * 5.8
            ) // total ticks
            / 696
        ) // viper slide unfolded length
    );
    public static final int ROBOT_TICKS_PER_MM = 2000 // ticks
            /1205; // mm

}
