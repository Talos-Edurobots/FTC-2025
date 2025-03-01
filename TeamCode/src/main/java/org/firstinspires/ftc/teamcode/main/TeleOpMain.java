package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This is (mostly) the OpMode used in the goBILDA Robot in 3 Days for the 24-25 Into The Deep FTC Season.
 * https://youtube.com/playlist?list=PLpytbFEB5mLcWxf6rOHqbmYjDi9BbK00p&si=NyQLwyIkcZvZEirP (playlist of videos)
 * This is very much based on the code for the Starter Kit Robot for the 24-25 season. Those resources can be found here:
 * https://www.gobilda.com/ftc-starter-bot-resource-guide-into-the-deep/
 *
 * There are three main additions to the starter kit bot code, mecanum drive, a linear slide for reaching
 * into the submersible.
 *
 * the drive system is all 5203-2402-0019 (312 RPM Yellow Jacket Motors) and it is based on a Strafer chassis
 * The arm shoulder takes the design from the starter kit robot. So it uses the same 117rpm motor with an
 * external 5:1 reduction
 *
 * The drivetrain is set up as "field centric" with the internal control hub IMU. This means
 * when you push the stick forward, regardless of robot orientation, the robot drives away from you.
 * We "took inspiration" (copy-pasted) the drive code from this GM0 page
 */


@TeleOp(name="TeleOpMain", group="Robot")
//@Disabled
public class TeleOpMain extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor      leftFrontDrive; //the left front drivetrain motor
    public DcMotor      rightFrontDrive; //the right front drivetrain motor
    public DcMotor      leftBackDrive; //the left back drivetrain motor
    public DcMotor      rightBackDrive; //the right bacck drivetrain motor
    public DcMotor      armMotor; //the arm motor
    public DcMotor      viperMotor; // the viper slide motor
    public Servo        intake; //the active intake servo
    public Servo        wrist; //the wrist servo
    public SparkFunOTOS otos; // the optical odometry sensor

    /* Variables that are used to set the arm and viper to a specific position */
    int armPosition;
    int armPositionFudgeFactor;

    // these constants store the minimum and maximum values for the viper motor after initialization
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
    double straferSpeedFactor = 1.2;
    int viperCalibrationAmount = 100;

    @Override
    public void runOpMode() {

        // hardware initialization e.g. motors and sensors
        initializeIO();
        //optical odometry sensor initialization
        configureOtos();
        // Retrieve the IMU from the hardware map. Gyroscope initialization
        initializeIMU();

        /* Having finished initialization process. the program
        waits for the game driver to press play button in driver station console*/
        waitForStart();

        /* Runs continuously until the driver presses stop in driver station */
        while (opModeIsActive())
        {
            // driving in filed centric strafer mode
            straferMovement();
            // compensation factor in arm motor position
            setArmLiftComp();

            // reading current position of the optical odometry sensor
            pos = otos.getPosition();


            /* Gamepad 1 controls the robot's movement (strafing mode) and the positioning of the arm
            for hanging at the end of the game.
            Gamepad 2 controls all the arm positioning for scoring samples and specimens, and the Viper
            movement back and forth.
             */

            //  ---------------  Gamepad 2 Control --------------
            // Controlling wrist servo
            // wrist servo moves wrist to either horizontal or vertical position
            if (gamepad2.dpad_down) {
                wristVertical();
            }
            else if (gamepad2.dpad_up){
                // wrist horizontal
                wristHorizontal();
            }

            // // Controlling intake claw servo
            /*
            * our intake claw is always closed unless left_stick_y or right_stick_y of gamepad2 are
            * facing down or either right_stick_button and left_stick_button are pressed on gamepad2.
            */
//            if (gamepad2.left_stick_y < -0.05 || gamepad2.right_stick_y < -0.05 || gamepad2.right_stick_button || gamepad2.left_stick_button){
//                intakeOpen();
//            }
//            else {
//                intakeCollect();
//            }

            //Controlling arm positioning using buttons
            if(gamepad2.a){ // ps4: X
                /* This is the intake/ collecting arm position for collecting samples */
                armCollect();
               // viperCollapsed();
            }
            else if (gamepad2.b) { // ps4: O
                /*
                * This is the correct height to collect samples from the observation zone
                */
                armClearBarrier();
            }
            else if (gamepad2.y){ //ps4 triangle
                /* This is the correct height to score the sample in the HIGH BASKET */
                armScoreSampleInHigh();
            }
            else if (gamepad2.x) { //ps4 square
                /* moves the arm to an angle position for scoring specimens */
                armScoreSpecimen();
            }
            else if (gamepad2.dpad_left) {
                /* This is the starting configuration of the robot. This turns off and opens fully the intake,and moves the arm
                back to folded inside the robot. */
                armCollapsed();
                wristHorizontal();
                intakeOpen();
            }
            else if (gamepad2.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armScoreSampleInLow();
                wristHorizontal();
            }

            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                intakeOpen();
            }
            else {
                intakeCollect();
            }
            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it knows exactly where it is, and there's a limit to how far in and
            out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot like our arm. Where we click a button and it goes to a position, then stops.
            But the drivers wanted more "open loop" controls. So we want the lift to keep extending for
            as long as we hold the bumpers, and when we let go of the bumper, stop where it is at.
            This allows the driver to manually set the position, and not have to have a bunch of different
            options for how far out it goes. But it also lets us enforce the end stops for the slide
            in software. So that the motor can't run past it's endstops and stall.
            We have our liftPosition variable, which we increment or decrement for every cycle (every
            time our main robot code runs) that we're holding the button. Now since every cycle can take
            a different amount of time to complete, and we want the lift to move at a constant speed,
            we measure how long each cycle takes with the cycletime variable. Then multiply the
            speed we want the lift to run at (in mm/sec) by the cycletime variable. There's no way
            that our lift can move 2800mm in one cycle, but since each cycle is only a fraction of a second,
            we are only incrementing it a small amount each cycle.
             */
            if (gamepad2.left_stick_y <= -.25){
                viperPosition += (int) (2800 * cycleTime);
            }
            else if (gamepad2.left_stick_y >= .25){
                viperPosition -= (int) (2800 * cycleTime);
            }
            // Attention!! press these buttons ONLY if the viper motor has got stalled!
            //This will set current position as starting position for the viper and will
            // prevent the stall of the motor and its overheating
            // gamepad1 viper calibration position
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                calibrateViper();
            }

            /*
            * we normalize the viper motor position
            * we set the position as target position
            * we finally run the viper motor
            */
            viperNormalization();
            setViperTargetPosition();
            runViper();



            //  ---------------  Gamepad 1 Control --------------
            // Controlling strafing robot movement
            // controlling arm positions for hanging the robot

             if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armAttachHangingHook();
               // wristVertical();
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armWinchRobot();
                //wristVertical();
            }

            // handling arm's positioning
            configureFudge();
            setArmTargetPosition();
            runArm();



            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("ARM MOTOR EXCEEDED CURRENT LIMIT!!");
            }

            /* Check to see if our viper motor is over the current limit, and report via telemetry. */
            if (((DcMotorEx) viperMotor).isOverCurrent()) {
                telemetry.addLine("VIPER MOTOR EXCEEDED CURRENT LIMIT!!");
            }


            /* This is how we check our loop time. We create three variables:
            looptime is the current time when we hit this part of the code
            cycletime is the amount of time in seconds our current loop took
            oldtime is the time in seconds that the previous loop started at

            we find cycletime by just subtracting the old time from the current time.
            For example, lets say it is 12:01.1, and then a loop goes by and it's 12:01.2.
            We can take the current time (12:01.2) and subtract the oldtime (12:01.1) and we're left
            with just the difference, 0.1 seconds.

             */
            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;

            //requesting telemetry data
            output();
        }
    }

    //    ---------------- | initialization, output | ---------------------------------------------------------------------------------------------------------------------------------------------
    public void initializeIO() {
        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("left_front");
        leftBackDrive   = hardwareMap.dcMotor.get("left_back");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        rightBackDrive  = hardwareMap.dcMotor.get("right_back");
        viperMotor      = hardwareMap.dcMotor.get("viper_motor"); // linear viper slide motor
        armMotor        = hardwareMap.get(DcMotor.class, "dc_arm"); //the arm motor

        // define the optical odometry sensor object
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

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
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);

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

    public void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void output(){
        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addLine("Version: Android 5 orfanak");
        telemetry.addData("armMotor Current:",((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("viperMotor Current:",((DcMotorEx) viperMotor).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("arm target Position: ", armMotor.getTargetPosition());
        telemetry.addData("arm current position: ", armMotor.getCurrentPosition());
        //telemetry.addData("viper busy", viperMotor.isBusy());
        telemetry.addData("viper target Position", viperMotor.getTargetPosition());
        telemetry.addData("viper current position", viperMotor.getCurrentPosition());
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

    // ---------------- | arm position handling| ----------------------------------------------------------------------------------------------------------------------------------


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
    public void setArmLiftComp(){
            /*
            This is probably my favorite piece of code on this robot. It's a clever little software
            solution to a problem the robot has.
            This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
            run to a specific angle, and stop there to collect from the field. And the angle that
            the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
            so here, we add a compensation factor based on how far the lift is extended.
            That comp factor is multiplied by the number of mm the lift is extended, which
            results in the number of degrees we need to fudge our arm up by to keep the end of the arm
            the same distance from the field.
            Now we don't need this to happen when the arm is up and in scoring position. So if the arm
            is above 45°, then we just set armLiftComp to 0. It's only if it's below 45° that we set it
            to a value.
             */

        if (armPosition < armDegreesToTicks(45)) {
            armLiftComp = 0; // 0.25568
        }
        else {
            armLiftComp = 0;
        }
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
        armPosition = armDegreesToTicks(105);
    }
    public void armWinchRobot() {
        armPosition = armDegreesToTicks(0);
    }
    public void configureFudge() {
            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */
        armPositionFudgeFactor = (int) (
                (armDegreesToTicks(10) * gamepad2.right_trigger) +
                (armDegreesToTicks(15) * (-gamepad2.left_trigger))
        );
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
   // public void viperScoreInLow() {
   //     viperPosition = viperMotorMmToTicks(0);
   //     viperPosition = viperMotorMmToTicks(0);
  //  }
   // public void viperScoreInHigh() {
   //     viperPosition = viperMotorMmToTicks(380);
  //  }
    public void viperCollapsed() {
        viperPosition = viperMotorMmToTicks(0);
    }
    //    public void viperDeltaTimeIncrement() {
//        viperPosition += (int) (5000 * cycleTime); // 3600
//    }
//    public void viperDeltaTimeDecrement() {
//        viperPosition -= (int) (5000 * cycleTime); // 3600
//    }
   // public void viperDeltaTime() {
     //   viperPositionDelta -= (int) ((int) (2000 * cycleTime) * (gamepad2.right_stick_y + gamepad2.left_stick_y)); // 2000
    //   // viperPositionDelta -= (int) ((int) (2000 * cycleTime) * (gamepad2.right_stick_y + gamepad2.left_stick_y)); // 2000
  //  }

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

    //    ---------------- | strafer movement| --------------------------------------------------
    public void straferMovement(){
        double y = -gamepad1.left_stick_y; //
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        // press options button to reset IMU
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // we multiply denominator variable by a variable named "straferSpeedFactor" with a value greater than 1
        // in order to reduce the strafing speed. The normal strafing speed is to high and thus difficult to control the robot
        double denominator = straferSpeedFactor* Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
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
        otos.setLinearScalar(1.138); //known distance 182cm, measured distance 160cm, error 182/160 = 1.138
        otos.setAngularScalar(0.9978); // 10 rotations 3600 degrees, measured 3608, error 3600/3608 =0.9978

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
}


