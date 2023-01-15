package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

public class mecanumDrive {
    // Note: we make these public so the calling code can access and use these variables
    public final DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private List<DcMotor> motors;
    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

    public final Rev2mDistanceSensor distanceSensorRight, distanceSensorLeft;
    public double distanceLeft, distanceRight;

    // Used to track slow-mode versus normal mode
    public Speeds currentSpeed;
    public enum Speeds {
        SLOW,
        NORMAL
    }
    
    public enum centerModes {
        CONE,
        STACK
    }

    public mecanumDrive(HardwareMap hardwareMap) {
        // set default value for speed
        currentSpeed = Speeds.NORMAL;

        // Set the drive motor variables based on hardware config
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightBackDrive, rightFrontDrive);

        // Set the direction of the motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set default power for motors
        leftFrontPower = 0.5;
        leftBackPower = 0.5;
        rightFrontPower = 0.5;
        rightBackPower = 0.5;

        // set sensor variables based on hardware config
        distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_right");
        distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_left");

        // initialize the distance measurements using centimeters units
        distanceLeft = distanceSensorLeft.getDistance(DistanceUnit.CM);
        distanceRight = distanceSensorRight.getDistance(DistanceUnit.CM);

    }

    // Updates power to the 4 drive motors based on input from the stick on the first controller
    public void updateMotorsFromStick(Gamepad gamepad) {
        float speedMultiplier;
        double vertical, horizontal, turn;
        double max;

        // Set the default value of speedMultiplier for NORMAL mode
        speedMultiplier = 1f;

        // Set the speedMultiplier in case of SLOW mode
        if (currentSpeed == Speeds.SLOW) {
            speedMultiplier = 0.75f;
        }

        // Assign human readable names to the stick inputs
        // Note: Pushing the stick forward gives a negative value, so we have to invert it
        vertical = -gamepad.left_stick_y;
        horizontal = gamepad.left_stick_x;
        turn = gamepad.right_stick_x;
        turn = turn * .85;

        // Calculate individual motor power base on the stick input values
        leftFrontPower = vertical + horizontal + turn;
        rightFrontPower = vertical - horizontal - turn;
        leftBackPower = vertical - horizontal + turn;
        rightBackPower = vertical + horizontal - turn;

        // Normalize the power values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Set the power on the motors
        leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
        rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
        leftBackDrive.setPower(leftBackPower * speedMultiplier);
        rightBackDrive.setPower(rightBackPower * speedMultiplier);
    }

    // Updates power to the 4 drive motors based on input from the stick on the first controller
    public void updateMotorsFromDpad(Gamepad gamepad) {
        double power;

        // Set default value for power
        power = 1f;

        // Set the power to send to the motors based on currentSpeed setting
        if (currentSpeed == Speeds.SLOW) {
            power = 0.75f;
        }

        if (gamepad.dpad_right) {
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(power * -1);
            rightFrontDrive.setPower(power * -1);
        } else if (gamepad.dpad_left) {
            leftFrontDrive.setPower(power * -1);
            rightBackDrive.setPower(power * -1);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        } else if (gamepad.dpad_up) {
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        } else if (gamepad.dpad_down) {
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
        }
    }

    // Changes the drive speed mode to slow
    public void setSlowMode() {
        currentSpeed = Speeds.SLOW;
    }

    // Change the drive speed mode to normal
    public void setNormalMode() {
        currentSpeed = Speeds.NORMAL;
    }

    // runs the centering code once
    // for use in driver control opmode where drive is holding down a button to auto-center
    public Boolean autoCenter(centerModes mode) {
        return center(mode);
    }

    // runs the centering code in a loop until it is centered or it times out
    // for use in autonomous opmode
    public Boolean autoCenterLoop(centerModes mode) {
        Boolean centered = false;
        // set timeout for breaking out of the loop (in milliseconds)
        long timeout = 3000;
        long start, current, elapsed;

        // need to set zero power behavior to brake for more accuracy
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        start = System.currentTimeMillis();
        while (!centered) {
            current = System.currentTimeMillis();
            elapsed = current - start;
            if (elapsed > timeout) {
                break;
            }
            centered = center(mode);
        }
        return centered;
    }

    // common centering code used for both autoCenter and autoCenterLoop
    private Boolean center(centerModes mode) {
        double startCentering, finishCentering, stopDistance;

        // default values are for centerModes.CONE
        startCentering = 30;
        finishCentering = 14;
        stopDistance = 8;

        // get a fresh copy of the current sensor readings
        distanceLeft = distanceSensorLeft.getDistance(DistanceUnit.CM);
        distanceRight = distanceSensorRight.getDistance(DistanceUnit.CM);

        if (distanceLeft <= stopDistance && distanceRight <= stopDistance) {
            // if both sides are < target stopping distance, stop and return true
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            return true;
        } else if (distanceLeft <= finishCentering && distanceRight <= finishCentering) {
            // if we are centering on a CONE over a pole, we now drive forward a fixed distance
            if (mode == centerModes.CONE) {
                leftFrontDrive.setPower(0.4);
                rightBackDrive.setPower(0.4);
                leftBackDrive.setPower(0.4);
                rightFrontDrive.setPower(0.4);
            // otherwise we stop, return true and let the user code drive forward correct distance
            } else {
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                return true;
            }
        } else if (distanceRight <= finishCentering) {
            // if only right side is < target centering distance, drive right
            leftFrontDrive.setPower(0.3);
            rightBackDrive.setPower(0.3);
            leftBackDrive.setPower(-0.3);
            rightFrontDrive.setPower(-0.3);
        } else if (distanceLeft <= finishCentering) {
            // if only the left side is < target centering distance, drive left
            leftFrontDrive.setPower(-0.3);
            rightBackDrive.setPower(-0.3);
            leftBackDrive.setPower(0.3);
            rightFrontDrive.setPower(0.3);
        } else if ((distanceRight <= startCentering && distanceRight > finishCentering) || (distanceLeft <= startCentering && distanceLeft > finishCentering)) {
            // if either side is between target centering distance <> start distance, drive forward
            leftFrontDrive.setPower(0.4);
            rightBackDrive.setPower(0.4);
            leftBackDrive.setPower(0.4);
            rightFrontDrive.setPower(0.4);
        }
        return false;
    }

    // used to change the zero power behavior between BRAKE and FLOAT
    public Boolean driveUntilClosed(liftArm arm) {
        long timeout = 3000;
        long start, current, elapsed;

        if (arm.grabber_status == liftArm.grabberPositions.CLOSED) {
            return false;
        }

        start = System.currentTimeMillis();
        while (arm.grabber_status == liftArm.grabberPositions.OPEN) {
            if (arm.color.red() > 400 || arm.color.blue() > 400) {
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                arm.closeGrabber();
                return true;
            } else {
                leftFrontDrive.setPower(0.4);
                rightBackDrive.setPower(0.4);
                leftBackDrive.setPower(0.4);
                rightFrontDrive.setPower(0.4);
            }
            current = System.currentTimeMillis();
            elapsed = current - start;
            if (elapsed > timeout) {
                break;
            }
        }
        return false;
    }

    // used to change the zero power behavior between BRAKE and FLOAT
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void stop() {
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
}


