package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutonomousMain", group="Robot")
public class AutonomousMain extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor leftFront; //the left front drivetrain motor
    public DcMotor rightFront; //the right front drivetrain motor
    public DcMotor leftBack; //the left back drivetrain motor
    public DcMotor rightBack; //the right bacck drivetrain motor
    public DcMotor armMotor; //the arm motor
    public DcMotor viperMotor; // the viper slide motor
    public Servo intake; //the active intake servo
    public Servo wrist; //the wrist servo
    public SparkFunOTOS otos; // the optical odometry sensor

    /* Variables that are used to set the arm and viper to a specific position */
    int armPosition;
    int armPositionFudgeFactor;

    //These constants store the minimum and maximum values for the viper motor after initialization
    final int MAX_VIPER_POSITION = viperMotorMmToTicks(480); // max viper extension (48 cm)
    final int MIN_VIPER_POSITION = viperMotorMmToTicks(0); // min viper extension (0 cm)
    int viperPosition; // store the current position of the viper motor in ticks
    int viperPositionDelta;
    int armLiftComp = 0; // store the compensation factor for the arm lift
    IMU imu; // the IMU sensor object
    int capturedViperPosition;
    boolean wristVertical;
    boolean intakeOpened;
    // loop cycle time variants
    double cycleTime = 0;
    double loopTime = 0;
    double oldTime = 0;
    //odometry sensor
    SparkFunOTOS.Pose2D pos;
    // strafer speed compensation factor
    double straferSpeedFactor = 1.5;
    int viperCalibrationAmount = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        // hardware initialization e.g. motors and sensors
        initializeIO();
        //optical odometry sensor initialization
        configureOtos();
        // Retrieve the IMU from the hardware map. Gyroscope initialization
        initializeIMU();
        // waiting for the players to hit start
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        // Autonomous operation
        while (opModeIsActive()) {
            gotoPosition(100, 100);
        }
        /* Handling viper's position
         * we normalize the viper motor position
         * we set the position as target position
         * we finally run the viper motor
         */
        viperNormalization();
        setViperTargetPosition();
        runViper();

        // handling arm's positioning
        setArmTargetPosition();
        runArm();
    }

    // -------------- Functions ------------------------------

    public void gotoPosition(double x, double y) {
        SparkFunOTOS.Pose2D currentPosition = otos.getPosition();
        double dx = x - currentPosition.x;
        double dy = y - currentPosition.y;
        double rx = 0;


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = dx * Math.cos(-botHeading) - dy * Math.sin(-botHeading);
        double rotY = dx * Math.sin(-botHeading) + dy * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = straferSpeedFactor * Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX) / denominator;
        double backLeftPower = (rotY - rotX) / denominator;
        double frontRightPower = (rotY - rotX) / denominator;
        double backRightPower = (rotY + rotX) / denominator;

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);
        telemetry.update();

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        otos.setLinearUnit(DistanceUnit.MM);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        pos = otos.getPosition();

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
//        telemetry.addData("X coordinate", pos.x);
//        telemetry.addData("Y coordinate", pos.y);
//        telemetry.addData("Heading angle", pos.h);
        telemetry.update();
    }
    public void initializeIO() {
    /* Define and Initialize Motors */
    leftFront =hardwareMap.dcMotor.get("left_front");
    leftBack =hardwareMap.dcMotor.get("left_back");
    rightFront =hardwareMap.dcMotor.get("right_front");
    rightBack =hardwareMap.dcMotor.get("right_back");
    viperMotor      =hardwareMap.dcMotor.get("viper_motor"); // linear viper slide motor
    armMotor        =hardwareMap.get(DcMotor .class,"dc_arm"); //the arm motor

    // define the optical odometry sensor object
    otos =hardwareMap.get(SparkFunOTOS .class,"otos");

       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx)armMotor).

    setCurrentAlert(5,CurrentUnit.AMPS);

    /*This sets the maximum current that the control hub will apply to the viper motor before throwing a flag */
        ((DcMotorEx)viperMotor).

    setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the arm and viper motors. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */

        armMotor.setTargetPosition(0);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setTargetPosition(0);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    /* Define and initialize servos.*/
    intake =hardwareMap.get(Servo .class,"intake_servo");
    wrist  =hardwareMap.get(Servo .class,"wrist_servo");

    /* Starting position with the wrist horizontal and intake open*/
    wristHorizontal();

    intakeOpen();

    /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
}
// IMU initialization
public void initializeIMU() {
    imu = hardwareMap.get(IMU.class, "imu");
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    imu.initialize(parameters);
}

// Telemetry output
public void output(){
    /* send telemetry to the driver of the arm's current position and target position */
    telemetry.addLine("Version: Autonomous Android 1 orfanak");
    telemetry.addData("arm target Position: ", armMotor.getTargetPosition());
    telemetry.addData("arm current position: ", armMotor.getCurrentPosition());
    telemetry.addData("armMotor Current:",((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("viper busy", viperMotor.isBusy());
    telemetry.addData("viper target Position", viperMotor.getTargetPosition());
    telemetry.addData("viper current position", viperMotor.getCurrentPosition());
    telemetry.addData("viperMotor Current:",((DcMotorEx) viperMotor).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("cycle time sec",cycleTime);
    telemetry.addData("wrist pos", wrist.getPosition());
    telemetry.addData("intake pos", intake.getPosition());
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
    telemetry.addData("X coordinate", pos.x);
    telemetry.addData("Y coordinate", pos.y);
    telemetry.addData("Heading angle", pos.h);
    telemetry.update();
}

public int armDegreesToTicks(double degrees) {
    /* this function converts degrees to ticks for the arm motor */
    return (int) (
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0 // we want ticks per degree, not per rotation
                    * degrees // the specified degrees
    );

}
public void setArmPosition(int degrees) {
    armPosition = degrees;
}
public void armCollapsed() {
    armPosition = 0;
}
public void armClearBarrier() {
            /* This is about 20° up from the collecting position to clear the barrier
            Note here that we don't set the wrist position or the intake power when we
            select this "mode", this means that the intake and wrist will continue what
            they were doing before we clicked left bumper. */
    armPosition = armDegreesToTicks(20);
}

// these are functions for arm movement
public void armCollect(){
    armPosition = armDegreesToTicks(10);
}
public void armScoreSpecimen() {
    armPosition = armDegreesToTicks(85);
}
public void armScoreSampleInHigh() {
    armPosition = armDegreesToTicks(110);
} // 90
public void armAttachHangingHook() {
    armPosition = armDegreesToTicks(120);
}

public void armScoreSampleInLow() {
    armPosition = armDegreesToTicks(95);
}

public void setArmTargetPosition() {
           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

    //
    armMotor.setTargetPosition(armPosition + armPositionFudgeFactor + armLiftComp);
}

public void runArm() {
    ((DcMotorEx) armMotor).setVelocity(2500); // 2500
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
}

//    ---------------- | intake system | -----------------------------------------------------------

public void intakeOpen() {
    intake.setPosition(0); // intake open
    intakeOpened = true;
}
public void intakeCollect() {
    intake.setPosition(1);
}
public void wristVertical() {
    wrist.setPosition(0.60); // 0.6
    wristVertical = true;
}
public void wristHorizontal() {
    wrist.setPosition(0);
    wristVertical = false;
}

//    ---------------- | viper slide | -------------------------------------------------------------
public int viperMotorMmToTicks(int mm) {
    /*
     * 312 rpm motor: 537.7 ticks per revolution
     * 4 stage viper slide (240mm): 5,8 rotations to fully expand
     * max travel distance: 696mm
     * ticks per mm = (537,7 * 5,8) ticks / (696) mm = 4,48 ticks / mm
     */
    return (int)
            (
                    (
                            (
                                    537.7 * 5.8
                            ) // total ticks
                                    / 696
                    ) // viper slide unfolded length
                            * mm // specified length

            ) + viperCalibrationAmount; //we add 100mm at the end because our viper is not able to completely fold inside (i.e. go to 0mm) while the viper motor continues to try
    // to achieve its target 0mm positionn. This has the result the motor to heat up and get stalled and get destroyed. However the viper motor always achieves the target for
    //100mm position and thus doesn't get streesed.



}
public void setViperPosition(int mm) {
    viperPosition = viperMotorMmToTicks(mm);
}

public void viperCollapsed() {
    viperPosition = viperMotorMmToTicks(0);
}

public void viperNormalization() {
        /*here we check to see if the lift is trying to go higher than the maximum extension.
           if it is, we set the variable to the max. */

    if (viperPosition > MAX_VIPER_POSITION){
        viperPosition = MAX_VIPER_POSITION;
    }
    // same as above, we see if the lift is trying to go below the minimum limit, and if it is, we set it to 0.
    if (viperPosition < MIN_VIPER_POSITION) {
        viperPosition = MIN_VIPER_POSITION;
    }

}
public void setViperTargetPosition(){
    viperMotor.setTargetPosition(viperPosition);
}
public void calibrateViper() {
    viperCalibrationAmount += viperMotor.getCurrentPosition();
}
public void runViper() {
    ((DcMotorEx) viperMotor).setVelocity(3000); //velocity of the viper slide
    viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
}
}