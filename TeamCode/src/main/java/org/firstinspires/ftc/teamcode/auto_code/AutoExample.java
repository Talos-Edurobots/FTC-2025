package org.firstinspires.ftc.teamcode.auto_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility_code.DriveTo;
import org.firstinspires.ftc.teamcode.utility_code.Stampede;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;

// orf
import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//orf

@Autonomous(name = "AUTO", group = "Autonomous")
public class AutoExample extends OpMode {
    boolean isRed = true;
    boolean isAudience = true;
    boolean recentIsRedChange = false;
    boolean recentAudienceChange = false;
    DriveTo driveTo;
    Stampede stampede;
    // This is the FIRST state for the State Machine
    String nextState = "actionStart";
    // We'll set this when we need to wait for an action to complete rather than check if the lift or drive is busy.
    double wait = 0;


    // For where coordinates are on the field for our different auto modes (diff start positions, ect.)
    HashMap<String, double[]> drivePositionsAudienceRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsAudienceBlue = new HashMap<>();
    HashMap<String, double[]> drivePositionsBackRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsBackBlue = new HashMap<>();
    HashMap<String, double[]> drivePositions;

    // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.



    @Override
    public void init() {
        initializeIO(); // orf
        stampede = new Stampede();
        stampede.init(hardwareMap);

        driveTo = new DriveTo(stampede, telemetry);

        //x, y, heading for start positions
        drivePositionsAudienceRed.put("start", new double[]{-12, -63, 90});
        drivePositionsAudienceBlue.put("start", new double[]{-12, 63, -90});
        drivePositionsBackRed.put("start", new double[]{12, -63, 90});
        drivePositionsBackBlue.put("start", new double[]{12, 63, -90});

        drivePositionsAudienceRed.put("Position 1", new double[]{-36, -40, 90});
        drivePositionsAudienceBlue.put("Position 1", new double[]{-36, 40, -90});
        drivePositionsBackRed.put("Position 1", new double[]{12, -40, 90});
        drivePositionsBackBlue.put("Position 1", new double[]{36, 40, -90});

        drivePositionsAudienceRed.put("Position 2", new double[]{24, -48, 135});
        drivePositionsAudienceBlue.put("Position 2", new double[]{-48, 60, 0});
        drivePositionsBackRed.put("Position 2", new double[]{48, -60, 0});
        drivePositionsBackBlue.put("Position 2", new double[]{-24, 48, 135 + 180});

        drivePositionsAudienceRed.put("Position 3", new double[]{48, -60, 135});
        drivePositionsAudienceBlue.put("Position 3", new double[]{-48, -48, 0});
        drivePositionsBackRed.put("Position 3", new double[]{48, -48, 0});
        drivePositionsBackBlue.put("Position 3", new double[]{-48, 60, 135 + 180});
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {        //This is our select start position
            if (!recentIsRedChange) {
                isRed = !isRed;
                recentIsRedChange = true;
            }
        } else {
            recentIsRedChange = false;
        }
        if (gamepad1.dpad_left) {      //This is our select audience position
            if (!recentAudienceChange) {
                isAudience = !isAudience;
                recentAudienceChange = true;
            }
        } else {
            recentAudienceChange = false;
        }

        telemetry.addData("Team (up)", "%s", isRed ? "RED" : "BLUE");
        telemetry.addData("Position (left)", "%s", isAudience ? "Audience" : "Back");
    }

    @Override
    public void start() {
        //Pick from our hashmap for color and side
        if (isAudience && isRed) {
            drivePositions = drivePositionsAudienceRed;
        } else if (isAudience && !isRed) {
            drivePositions = drivePositionsAudienceBlue;
        } else if (!isAudience && isRed) {
            drivePositions = drivePositionsBackRed;
        } else {
            drivePositions = drivePositionsBackBlue;
        }

        //setting start position, [0] is x, [1] is y, [2] is heading
        stampede.xFieldPos = drivePositions.get("start")[0];
        stampede.yFieldPos = drivePositions.get("start")[1];
        stampede.headingField = drivePositions.get("start")[2];
        stampede.angleTracker.setOrientation(stampede.headingField);
    }

    @Override
    public void loop() {
        stampede.updateFieldPosition();
        telemetry.addData("Field Position (Coordinates)", "%.2f, %.2f, %.2f", stampede.xFieldPos, stampede.yFieldPos, stampede.headingField);
        telemetry.addData("IMU Orientation", "IMU %.2f", stampede.angleTracker.getOrientation());
        telemetry.addData("Next action", nextState);

        driveTo.sendTelemetry(telemetry);
        driveTo.updateDrive();

        if (!isBusy()) {
            // Variable used in getDeclaredMethod cannot be changed within the loop because it is being used,
            // so we create another variable so that we may change nextState!
            String currentState = nextState;
            try {
                // Inspect our own class to see if we have an action method with the name we specified.

                Method stateMethod = this.getClass().getMethod(currentState, new Class[]{});
                // Invoke it with our class instance.
                stateMethod.invoke(this);
                // Catch exceptions to keep the compiler happy.
            } catch (NoSuchMethodException exc) {
                // This will be caught when we haven't defined the method (e.g., for "done").
            } catch (IllegalAccessException exc) {
                // We don't expect this one.
                telemetry.addData("exception", "IllegalAccessException when calling " + currentState + " " + exc);
            } catch (InvocationTargetException exc) {
                // We don't expect this one.
                telemetry.addData("exception", "InvocationTargetException when calling " + currentState + " " +
                        exc.getTargetException() + " " + exc.getTargetException().getStackTrace()[0]);
            }
        }
    }

    @Override
    public void stop() {
        stampede.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
    }

    // Conditions that the robot is busy in.
    Boolean isBusy() {
        // If robot isn't there yet its busy.
        if (!driveTo.areWeThereYet) {
            return true;
        }
        /*
        You can check for other busy conditions like this (e.g., for any other motors you want to move in auto).

        if (robot.isLiftBusy()) {
            return true;
        }
        */
        if (getRuntime() < wait) {
            return true;
        }
        return false;
    }

    // This is the State Machine, it's the "steps" the robot will follow.
    public void actionStart() {
        driveTo.setTargetPosition(drivePositions.get("Position 1"), .25);
        // This is how you can add a wait.
        //wait = getRuntime() + 5;
        // Name what the next action should be.
        nextState = "actionStep2";
    }

    public void actionStep2() {
        // stopBetween is whether the robot will stop between positions, or just drive through the position.
        driveTo.setTargetPosition(drivePositions.get("Position 2"), .25, false);
        nextState = "actionStep3";
    }

    public void actionStep3() {
        driveTo.setTargetPosition(drivePositions.get("Position 3"), .5);
        nextState = "actionStop";
    }

    public void actionStop() {
        stampede.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
        nextState = "actionDone";
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

    public void initializeIO() {
        /* Define and Initialize Motors */
    
        viperMotor      = hardwareMap.dcMotor.get("viper_motor"); // linear viper slide motor
        armMotor        = hardwareMap.get(DcMotor.class, "dc_arm"); //the arm motor
      //  otos            = hardwareMap.get(SparkFunOTOS.class, "otos");
        
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

}
