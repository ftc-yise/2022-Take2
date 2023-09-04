package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class liftArm2 {
    // Define hardware instance variables here
    public final Servo poleSupport;
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

    public liftArm2(HardwareMap hardwareMap) {
        // initialize slide motors


        // initialize cone grabber

        // initialize pole positioning arm
        poleSupport = hardwareMap.get(Servo.class, "pole_support");
        poleSupport.setPosition(0.5);
        pole_status = polePositions.UP;

        // initialize claw slider



    }

    // moves the arm to a predefined height/position


    // resets the arm back to bottom based on encoder position of 0


    // used in driver control to forcearm back to the bottom overriding the encoder position


    // used in driver control to reset encoder to postion 0 after forcing down







    // Sets the lift arm height for grabbing the top cone of the 5 cone stack






    public void poleUp() {
        poleSupport.setPosition(0.56);
        pole_status = polePositions.UP;
    }
    public void poleDown() {
        poleSupport.setPosition(0.25);
        pole_status = polePositions.DOWN;
    }



}