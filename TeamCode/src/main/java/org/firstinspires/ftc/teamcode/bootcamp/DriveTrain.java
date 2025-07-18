package org.firstinspires.ftc.teamcode.bootcamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 *
 */
public class DriveTrain {
    private DcMotor left, right;
    private double normalSpeed = .8; // the speed that the robot will
    private double slowSpeed = .4;
    private double gainInSpeedPerLoop = .001; // the time in seconds that the robot will take to go from fully stopped to max
    private double currentSpeed = 0; // the current speed of the robot
    Telemetry tel;
    public DriveTrain(Telemetry tel, HardwareMap hwmap, String leftName, String rightName) {
        this.tel = tel;
        left = hwmap.dcMotor.get(leftName);
        right = hwmap.dcMotor.get(rightName);
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void update(double forwardSpeed, double turnSpeed, double decelerationAmount, double deltaTime) {
        double targetStrafingSpeed = normalSpeed - (decelerationAmount * (normalSpeed - slowSpeed));

        right.setPower(targetStrafingSpeed*(forwardSpeed - turnSpeed));
        left.setPower(targetStrafingSpeed*(forwardSpeed + turnSpeed));

        tel.addData("target strafing speed", targetStrafingSpeed);
        tel.addData("current speed", currentSpeed);
        tel.addData("delta time", deltaTime);
        tel.addData("left drive power", left.getPower());
        tel.addData("right drive power", right.getPower());
    }

    public double getNormalSpeed() {
        return normalSpeed;
    }

    public void setNormalSpeed(double normalSpeed) {
        this.normalSpeed = normalSpeed;
    }

    public double getSlowSpeed() {
        return slowSpeed;
    }

    public void setSlowSpeed(double slowSpeed) {
        this.slowSpeed = slowSpeed;
    }
}
