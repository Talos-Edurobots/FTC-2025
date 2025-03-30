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
    public static final String VIPER_MOTOR_CONFIGURATION = "viper_motor";
    public static final String ARM_MOTOR_CONFIGURATION = "dc_arm";
    public static final String LEFT_FRONT_MOTOR_CONFIGURATION = "left_front";
    public static final String LEFT_BACK_MOTOR_CONFIGURATION = "left_back";
    public static final String RIGHT_FRONT_MOTOR_CONFIGURATION = "right_front";
    public static final String RIGHT_BACK_MOTOR_CONFIGURATION = "right_back";
    public static final String OTOS_CONFIGURATION = "otos";
    public static final String IMU_CONFIGURATION = "imu";
    public static final int ROBOT_HEIGHT_MM = 340;
    public static final int VIPER_COLLAPSED_LENGTH = 480;
    public static final int VIPER_EXTENDED_LENGTH = 696;
    public static final int ARM_VELOCITY = 2500;
    public static final int WHEEL_DIAMETER_MM = 96;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_MM * Math.PI;
    public static final double STRAFER_MOTORS_RPM = 312.0;
    public static final double STRAFER_MAX_SPEED = (WHEEL_CIRCUMFERENCE/1000.0)/(STRAFER_MOTORS_RPM/60.0); // m/s
    public static final double STRAFER_MOTORS_COUNTS_PER_REVOLUTION = 537.7;

}
