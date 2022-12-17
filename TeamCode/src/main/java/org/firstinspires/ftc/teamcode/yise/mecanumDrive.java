package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumDrive {
    // Note: we make these public so the calling code can access and use these variables
    public final DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

    // Used to track slow-mode versus normal mode
    public Speeds currentSpeed;
    public enum Speeds {
        SLOW,
        NORMAL
    }

    public mecanumDrive(HardwareMap hardwareMap) {
        // set default value for speed
        currentSpeed = Speeds.NORMAL;

        // Set the drive motor variables based on hardware config
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

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

}


