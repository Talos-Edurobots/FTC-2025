package org.firstinspires.ftc.teamcode.hardware.units.velocity;

import org.firstinspires.ftc.teamcode.hardware.Constants;

public class StraferMotorVelocity {

    private double mps, rpm, p;
    public enum Units {
        METERS_PER_SECOND,
        REVOLUTIONS_PER_MINUTE,
        POWER
    }
    StraferMotorVelocity(double value, Units unit) {
        if (unit == Units.METERS_PER_SECOND) {
            this.mps = value;
            this.rpm = (mps*60) / (Constants.WHEEL_CIRCUMFERENCE / 1000);
            this.p = rpm / Constants.STRAFER_MOTORS_RPM;
        }
        else if (unit == Units.REVOLUTIONS_PER_MINUTE) {
            this.rpm = value;
            this.mps = (rpm/60) * (Constants.WHEEL_CIRCUMFERENCE / 1000);
            this.p = rpm * Constants.STRAFER_MOTORS_RPM;
        }
        else if (unit == Units.POWER) {
            this.p = value;
            this.rpm = p * Constants.STRAFER_MOTORS_RPM;
            this.mps = (rpm/60) * (Constants.WHEEL_CIRCUMFERENCE / 1000);
        }
    }
    public double getP() {
        return this.p;
    }
    public double getMps() {
        return this.mps;
    }
    public double getRpm() {
        return this.rpm;
    }
}

