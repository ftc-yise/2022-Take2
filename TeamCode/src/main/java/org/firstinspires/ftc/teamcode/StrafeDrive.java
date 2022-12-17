package org.firstinspires.ftc.teamcode;

// opmode packages
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

// hardware packages
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

// other packages
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// team packages
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;
import org.firstinspires.ftc.teamcode.yise.liftArm;

@TeleOp(name="Drive program", group="Linear Opmode")
public class StrafeDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //private Rev2mDistanceSensor distanceSensorRight = null;
    //private Rev2mDistanceSensor distanceSensorLeft = null;

    private Servo coneGrabber = null;

    // state variables that track if buttons were released
    public boolean armResetButtonWasReleased = true;
    public boolean leftBumperWasReleased = true;
    public boolean canSwitchModes = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        mecanumDrive drive = new mecanumDrive(hardwareMap);

        // create instance of lift arm class
        liftArm arm = new liftArm(hardwareMap);

        coneGrabber = hardwareMap.get(Servo.class, "cone_grabber");

        //distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_right");
        //distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_left");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
            /*
            if (gamepad1.y) {
                canSwitchModes = false;
            } else {
                canSwitchModes = true;
            }
            */

            // If we have any Dpad input, update the motor power based on Dpad (ie overright stick)
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }

            // Lift Arm 4 Position Code
            if (gamepad2.dpad_up) {
                arm.setPoleHeight(liftArm.Heights.HIGH);
            } else if (gamepad2.dpad_left) {
                arm.setPoleHeight(liftArm.Heights.MEDIUM);
            } else if (gamepad2.dpad_right) {
                arm.setPoleHeight(liftArm.Heights.LOW);
            } else if (gamepad2.x) {
                arm.setPoleHeight(liftArm.Heights.HOVER);
            } else if (gamepad2.dpad_down) {
                arm.returnToBottom();
            }

            // Force lift arm down (ignoring encoders) - temp until limit switch integrated
            if ((gamepad1.right_stick_button || gamepad2.right_stick_button) && armResetButtonWasReleased){
                armResetButtonWasReleased = arm.forceDown();
            } else if ((!gamepad1.right_stick_button || !gamepad2.right_stick_button)  && !armResetButtonWasReleased) {
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
            if (!gamepad2.left_bumper){
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
            if (gamepad1.a || gamepad2.a) {
                coneGrabber.setPosition(Servo.MIN_POSITION);
            } else if (gamepad1.b || gamepad2.b) {
                coneGrabber.setPosition(Servo.MAX_POSITION);
            }

            // -----------------------------------------------------------------------------------
            // Auto-Centering Code
            // -----------------------------------------------------------------------------------
            //CAUSING LAG AND WHEN DISCONNECTS IT STOPS THE CODE BRING THIS OUT TO ANOTHER
            // CLASS AND ADD ERROR CODE HANDLING
            /*if (gamepad1.left_trigger >= 0.8) {
                    // if both sides are < 20cm, stop
                    if (distanceLeft < 20 && distanceRight < 20) {
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                    // if only right side is < 20cm, drive right
                    } else if (distanceRight < 20) {
                        leftFrontDrive.setPower(0.3);
                        rightBackDrive.setPower(0.3);
                        leftBackDrive.setPower(-0.3);
                        rightFrontDrive.setPower(-0.3);
                    // if only the left side is < 20cm, drive left
                    } else if (distanceLeft < 20) {
                        leftFrontDrive.setPower(-0.3);
                        rightBackDrive.setPower(-0.3);
                        leftBackDrive.setPower(0.3);
                        rightFrontDrive.setPower(0.3);
                    // if either side is between 20cm <> 35cm, drive forward
                    } else if ((distanceRight < 35 && distanceRight > 20) || (distanceLeft < 35 && distanceLeft > 20)) {
                        leftFrontDrive.setPower(0.4);
                        rightBackDrive.setPower(0.4);
                        leftBackDrive.setPower(0.4);
                        rightFrontDrive.setPower(0.4);
                    }
                }*/

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

            telemetry.update();
        }
    }
}