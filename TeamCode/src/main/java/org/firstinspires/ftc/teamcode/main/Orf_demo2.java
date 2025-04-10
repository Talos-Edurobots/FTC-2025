

package org.firstinspires.ftc.teamcode.main;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Orf_demo2", group="auto")
//@Disabled
public class Orf_demo2 extends LinearOpMode {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.112;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.03;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.0015;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members for each of the 4 drive motors.
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor armMotor;
    private DcMotor viperMotor;
    private Servo intake;
    private Servo wrist;
    private int armPosition;

    // Sensors
    private SparkFunOTOS otos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        initializeIO();
        configureOtos();
        sleep(100);

        while(!isStarted()) {
            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while(opModeIsActive() && (runtime.milliseconds() < 1000)){
            armPosition = armDegreesToTicks(20);
            setArmTargetPosition();
            runArm();
            sleep(100);
        }

        otosDrive(0, 100, 0, 2);      // small move forward and right away from wall
        otosDrive(30,100, 0, 2);     // forward and push sample into net zone
        otosDrive(30,100, 90, 2);
//        otosDrive(0, 24, 0, 2);     // backup and move away from wall
//        otosDrive(-87, 24, 0, 4);   // backup straight
//        otosDrive(-87, 4, 0, 2);    // park in observation zone

        sleep(10000);
    }

    public void initializeIO() {
        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("left_front");
        leftBackDrive   = hardwareMap.dcMotor.get("left_back");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        rightBackDrive  = hardwareMap.dcMotor.get("right_back");
        viperMotor      = hardwareMap.dcMotor.get("viper_motor"); // linear viper slide motor
        armMotor        = hardwareMap.get(DcMotor.class, "dc_arm"); //the arm motor
        otos            = hardwareMap.get(SparkFunOTOS.class, "otos");

        // define the optical odometry sensor object

       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /*This sets the maximum current that the control hub will apply to the viper motor before throwing a flag */
        ((DcMotorEx) viperMotor).setCurrentAlert(5,CurrentUnit.AMPS);

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
        intake = hardwareMap.get(Servo.class, "intake_servo");
        wrist  = hardwareMap.get(Servo.class, "wrist_servo");

        /* Starting position with the wrist horizontal and intake open*/
        wristHorizontal();
        intakeOpen();


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        otos.setLinearUnit(DistanceUnit.CM);
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
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0); // should be -3.75 & -7.5 and 90
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
        otos.setLinearScalar(1);
        otos.setAngularScalar(1);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
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

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
    void otosDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x; // dx
        yError = targetY-currentPos.y; // dy
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
                ((Math.abs(xError) > 1.5) || (Math.abs(yError) > 1.5) || (Math.abs(yawError) > 4)) ) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  =  Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe =  Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   =  Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.addData("arm target pos", armMotor.getTargetPosition());
        telemetry.addData("arm current pos", armMotor.getCurrentPosition());
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
    SparkFunOTOS.Pose2D myPosition() {
        pos = otos.getPosition();
        return(new SparkFunOTOS.Pose2D(pos.x, pos.y, pos.h));
    }
    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive X is forward
     * Positive Y is strafe right
     * Positive Yaw is clockwise: note this is not how the IMU reports yaw(heading)
     */
    void moveRobot(double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        sleep(10);
    }
    public void wristHorizontal() {
        wrist.setPosition(0);
    }
    public void wristVertical() {
        wrist.setPosition(0.60);
    }
    public void intakeOpen() {
        intake.setPosition(0); // intake open
    }
    public void intakeCollect() {
        intake.setPosition(1);
    }
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

                );
        // to achieve its target 0mm positionn. This has the result the motor to heat up and get stalled and get destroyed. However the viper motor always achieves the target for
        //100mm position and thus doesn't get streesed.
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
    public void setArmTargetPosition() {
        armMotor.setTargetPosition(armPosition);
    }
    public void runArm() {
        ((DcMotorEx) armMotor).setVelocity(2500); // 2500
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
    }

}
