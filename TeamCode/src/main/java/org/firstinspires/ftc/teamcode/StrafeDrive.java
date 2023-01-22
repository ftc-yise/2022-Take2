package org.firstinspires.ftc.teamcode;

// opmode packages
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// hardware packages
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

// other packages
import com.qualcomm.robotcore.util.ElapsedTime;

// team packages
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.ledLights;

@TeleOp(name="Drive program", group="Linear Opmode")
public class StrafeDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo coneGrabber = null;
    private ColorSensor color = null;

    // state variables that track if buttons were released
    public boolean armResetButtonWasReleased = true;
    public boolean leftBumperWasReleased = true;
    public boolean canSwitchModes = false;
    public boolean closed = false;
    public boolean idle = false;
    public boolean gamepadAWasReleased = true;

    @Override
    public void runOpMode() {

        // create instance of drive class
        mecanumDrive drive = new mecanumDrive(hardwareMap);

        // create instance of lift arm class
        liftArm arm = new liftArm(hardwareMap);

        ledLights leds = new ledLights(hardwareMap);

        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!idle) {
                leds.setLed(ledLights.ledStates.OPEN);
                idle = true;
            }

            // -----------------------------------------------------------------------------------
            // Drive Code
            // -----------------------------------------------------------------------------------

            // Handle changing between drive modes
            if (!gamepad1.y) {
                canSwitchModes = true;
            }
            if (gamepad1.y && (drive.currentSpeed == mecanumDrive.Speeds.NORMAL) && canSwitchModes) {
                drive.setSlowMode();
            } else if (gamepad1.y && (drive.currentSpeed == mecanumDrive.Speeds.SLOW) && canSwitchModes) {
                drive.setNormalMode();
            }

            if (gamepad1.y) {
                canSwitchModes = false;
            } else {
                canSwitchModes = true;
            }


            // If we have any Dpad input, update the motor power based on Dpad (ie overright stick)
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }

            // -----------------------------------------------------------------------------------
            // Lifm Arm Code
            // -----------------------------------------------------------------------------------
            if (gamepad2.dpad_up && closed) {
                arm.setPoleHeight(liftArm.Heights.HIGH);
                leds.setLed(ledLights.ledStates.HOVER);

            } else if (gamepad2.dpad_up && !closed) {
                arm.setPoleHeight(liftArm.Heights.HIGH);
                leds.setLed(ledLights.ledStates.BADHOVER);

            } else if (gamepad2.dpad_left && closed) {
                arm.setPoleHeight(liftArm.Heights.MEDIUM);
                leds.setLed(ledLights.ledStates.HOVER);

            } else if (gamepad2.dpad_left && !closed) {
                arm.setPoleHeight(liftArm.Heights.MEDIUM);
                leds.setLed(ledLights.ledStates.BADHOVER);
            } else if (gamepad2.dpad_right && closed) {
                arm.setPoleHeight(liftArm.Heights.LOW);
                leds.setLed(ledLights.ledStates.HOVER);

            } else if (gamepad2.dpad_right && !closed) {
                arm.setPoleHeight(liftArm.Heights.LOW);
                leds.setLed(ledLights.ledStates.BADHOVER);

            } else if (gamepad2.x && closed) {
                arm.setPoleHeight(liftArm.Heights.HOVER);
                leds.setLed(ledLights.ledStates.HOVER);
            } else if (gamepad2.x && !closed) {
                arm.setPoleHeight(liftArm.Heights.HOVER);
                leds.setLed(ledLights.ledStates.BADHOVER);
            } else if (gamepad2.dpad_down) {
                arm.returnToBottom();
                leds.setLed(ledLights.ledStates.OPEN);
            }

            // Pole Distancing Rod Code
            if (!gamepad1.a){
                gamepadAWasReleased = true;
            }
            if (gamepad1.a && gamepadAWasReleased) {
                gamepadAWasReleased = false;
                if (arm.pole_status == liftArm.polePositions.DOWN) {
                    arm.poleUp();
                } else if (arm.pole_status == liftArm.polePositions.UP) {
                    arm.poleDown();
                }
            }

            // Force lift arm down (ignoring encoders) - temp until limit switch integrated
            if ((gamepad1.right_stick_button || gamepad2.right_stick_button) && armResetButtonWasReleased) {
                armResetButtonWasReleased = arm.forceDown();
            } else if ((!gamepad1.right_stick_button || !gamepad2.right_stick_button) && !armResetButtonWasReleased) {
                armResetButtonWasReleased = arm.stopAndReset();
            }

            // Stop the slide and keep it from holding position
            if (!arm.slideStatusBusy() && armResetButtonWasReleased) {
                arm.holdPosition();
            }

            // ---------------------------------------------------------------------------
            // Cone Stack Code - Go up to cone 5 and step down 1 cone at a time
            // -----------------------------------------------------------------------------------

            // track when left_bumper is released so we only drop 1 position per button press
            if (!gamepad2.left_bumper) {
                leftBumperWasReleased = true;
            }

            // either drop 1 cone or go to top cone
            if (gamepad2.left_bumper && leftBumperWasReleased) {
                arm.downOneCone();
            } else if (gamepad2.right_bumper) {
                arm.getTopCone();
            }

            // -----------------------------------------------------------------------------------
            // Open and Close the Grabber
            // -----------------------------------------------------------------------------------
            if (color.red() > 400 && !closed) {
                arm.closeGrabber();
                leds.setLed(ledLights.ledStates.CLOSE);
                closed = true;
            } else if (color.blue() > 400 && !closed) {
                arm.closeGrabber();
                leds.setLed(ledLights.ledStates.CLOSE);
                closed = true;
            } else if (gamepad2.a && closed) {
                arm.openGrabber();
                leds.setLed(ledLights.ledStates.OPEN);
                closed = false;
            } else if (gamepad2.b && !closed) {
                arm.closeGrabber();
                leds.setLed(ledLights.ledStates.CLOSE);
                closed = true;
            }


            // -----------------------------------------------------------------------------------
            // Auto-Centering Code
            // -----------------------------------------------------------------------------------
            if (gamepad1.left_trigger >= 0.8) {
                drive.autoCenter(mecanumDrive.centerModes.CONE);
            } else if (gamepad1.right_trigger >= 0.8) {
                // drive.autoCenter(mecanumDrive.centerModes.POLE);
            }

            // -----------------------------------------------------------------------------------
            // Telemetry Code
            // -----------------------------------------------------------------------------------

            // Show the elapsed run time
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // Show the power of the drive motors
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFrontPower, drive.rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBackPower, drive.rightBackPower);

            // Show the position of the arm
            telemetry.addData("ArmHeightL: ", arm.getSlidePosition(liftArm.Sides.LEFT));
            telemetry.addData("ArmHeightR: ", arm.getSlidePosition(liftArm.Sides.RIGHT));
            telemetry.addData("Cone: ", arm.cone_position);

            // Add odometry wheel position
            telemetry.addData("Left Encoder:", drive.leftFrontDrive.getCurrentPosition());
            telemetry.addData("Rear Encoder:", drive.rightFrontDrive.getCurrentPosition());
            telemetry.addData(" Right Encoder:", -drive.rightBackDrive.getCurrentPosition());

            // Add distance sensor data
            //telemetry.addData("Distance left: ", distanceLeft);
            //telemetry.addData("Distance right: ", distanceRight);

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("pole", gamepadAWasReleased);

            //telemetry.addData("DistanceLeftV2: ", drive.distanceLeftV2);
            //telemetry.addData("DistanceRightV2: ", drive.distanceRightV2);

            telemetry.update();
        }
    }
}
