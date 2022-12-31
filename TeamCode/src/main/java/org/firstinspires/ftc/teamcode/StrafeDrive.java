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
                arm.openGrabber();
            } else if (gamepad1.b || gamepad2.b) {
                arm.closeGrabber();
            }

            // -----------------------------------------------------------------------------------
            // Auto-Centering Code
            // -----------------------------------------------------------------------------------
            if (gamepad1.left_trigger >= 0.8) {
                   drive.autoCenter(mecanumDrive.centerModes.CONE);
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

            telemetry.update();
        }
    }
}