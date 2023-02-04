package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class liftArm {
    // Define hardware instance variables here
    public final DcMotor leftSlide, rightSlide;
    public final Servo coneGrabber;
    public final ColorSensor color;
    public final Servo poleSupport;
    public final Servo clawSlide;

    // Tracks whether grabber is currently opened or closed
    public grabberPositions grabber_status;
    public enum grabberPositions {
        OPEN,
        CLOSED
    }

    // Tracks whether pole positioning arm is up or down
    public polePositions pole_status;
    public enum polePositions {
        DOWN,
        UP
    }

    // Tracks whether claw slide is in or out
    public clawSlidePositions clawSlide_status;
    public enum clawSlidePositions {
        IN,
        OUT
    }

    // Used to set pole height.
    public enum Heights {
        LOW,
        MEDIUM,
        HIGH,
        HOVER
    }

    // Used to identify left and right slide motors
    public enum Sides {
        RIGHT,
        LEFT
    }

    // tracks the height of the cone stack
    public int cone_position;

    public liftArm(HardwareMap hardwareMap) {
        // initialize slide motors
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // initialize cone grabber
        coneGrabber = hardwareMap.get(Servo.class, "cone_grabber");
        coneGrabber.setPosition(Servo.MIN_POSITION);
        grabber_status = grabberPositions.OPEN;

        // initialize pole positioning arm
        poleSupport = hardwareMap.get(Servo.class, "pole_support");
        poleSupport.setPosition(0.5);
        pole_status = polePositions.UP;

        // initialize claw slider
        clawSlide = hardwareMap.get(Servo.class, "claw_slide");
        clawSlide.setPosition(Servo.MIN_POSITION);
        clawSlide_status = clawSlidePositions.IN;

        // initialize color sensor
        color = hardwareMap.get(ColorSensor.class, "Color");
    }

    // moves the arm to a predefined height/position
    public void setPoleHeight(Heights targetHeight) {
        switch (targetHeight) {
            case LOW:
                leftSlide.setTargetPosition(850);
                rightSlide.setTargetPosition(850);
                break;
            case MEDIUM:
                leftSlide.setTargetPosition(1400);
                rightSlide.setTargetPosition(1400);
                break;
            case HIGH:
                leftSlide.setTargetPosition(1950);
                rightSlide.setTargetPosition(1950);
                break;
            case HOVER:
                leftSlide.setTargetPosition(75);
                rightSlide.setTargetPosition(75);
                break;
        }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    // resets the arm back to bottom based on encoder position of 0
    public void returnToBottom() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    // used in driver control to forcearm back to the bottom overriding the encoder position
    public boolean forceDown() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(-0.5);
        rightSlide.setPower(-0.5);
        return false;
    }

    // used in driver control to reset encoder to postion 0 after forcing down
    public boolean stopAndReset() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
    }

    public boolean slideStatusBusy() {
        boolean busy = false;
        if (rightSlide.isBusy() || leftSlide.isBusy()) {
            busy = true;
        }
        return busy;
    }

    public void holdPosition() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(0.05);
        rightSlide.setPower(0.05);
    }

    public void getConeFromStack(int targetCone) {
        int top_cone = 300;
        int cone_offset = 60;
        cone_position = top_cone - (cone_offset * (5 - targetCone));

        leftSlide.setTargetPosition(cone_position);
        rightSlide.setTargetPosition(cone_position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    // Sets the lift arm height for grabbing the top cone of the 5 cone stack
    public void getTopCone() {
        cone_position = 300;
        leftSlide.setTargetPosition(cone_position);
        rightSlide.setTargetPosition(cone_position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void downOneCone() {
        if (cone_position < 0) {
            cone_position = 1;
        } else {
            cone_position = cone_position - 70;  //controls how far the arm comes down on each press for each cone
            leftSlide.setTargetPosition(cone_position);
            rightSlide.setTargetPosition(cone_position);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
    }

    public double getSlidePosition(Sides side) {
        double position = 0;
        if (side == Sides.LEFT) {
            position = leftSlide.getCurrentPosition();
        } else if (side == Sides.RIGHT) {
            position = rightSlide.getCurrentPosition();
        }
        return position;
    }

    public void closeGrabber() {
        coneGrabber.setPosition(Servo.MAX_POSITION);
        grabber_status = grabberPositions.CLOSED;
    }
    public void openGrabber() {
        coneGrabber.setPosition(Servo.MIN_POSITION);
        grabber_status = grabberPositions.OPEN;
    }

    public void poleUp() {
        poleSupport.setPosition(0.5);
        pole_status = polePositions.UP;
    }
    public void poleDown() {
        poleSupport.setPosition(Servo.MAX_POSITION);
        pole_status = polePositions.DOWN;
    }

    public void clawOut() {
        clawSlide.setPosition(Servo.MAX_POSITION);
        clawSlide_status = clawSlidePositions.OUT;
    }
    public void clawIn() {
        clawSlide.setPosition(Servo.MIN_POSITION);
        clawSlide_status = clawSlidePositions.IN;
    }

    public Boolean findRed() {
        return color.red() > 400;
    }
    public Boolean findBlue() {
        return color.blue() > 400;
    }
}