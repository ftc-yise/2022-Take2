package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.yise.mecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Drive program", group="Linear Opmode")
public class StrafeDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    //private Rev2mDistanceSensor distanceSensorRight = null;
    //private Rev2mDistanceSensor distanceSensorLeft = null;

    private Servo coneGrabber = null;

    public int cone = 0;  // variable used to set the value for the cone heights and incremental down values

    boolean hasReset = true;

    public boolean canSwitchModes = true;

    public boolean leftBumperWasPressed = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        mecanumDrive drive = new mecanumDrive(hardwareMap);

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        coneGrabber = hardwareMap.get(Servo.class, "cone_grabber");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_right");
        //distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_left");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Handle changing between drive modes
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

            // If we have any Dpad input, update the motor power based on Dpad
            // Note: Dpad overrides the stick
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            // Otherwise update motor power based on stick input
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }

            // -----------------------------------------------------------------------------------
            // Lift Arm Positioning Code
            // -----------------------------------------------------------------------------------
            /*leftSlide.setPower(-gamepad2.left_stick_y);
            rightSlide.setPower(-gamepad2.left_stick_y);*/

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                leftSlide.setTargetPosition(1950); // high pole position based on string length
                rightSlide.setTargetPosition(1950);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                cone = 0;
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                leftSlide.setTargetPosition(850);  //low pole position
                rightSlide.setTargetPosition(850);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                leftSlide.setTargetPosition(1400);  //mid pole position
                rightSlide.setTargetPosition(1400);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad1.x || gamepad2.x) {
                leftSlide.setTargetPosition(75);  //GROUND JUNCTION hover
                rightSlide.setTargetPosition(75);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }

            // -----------------------------------------------------------------------------------
            // Cone Stack Code - Go up to core 5 and step down 1 cone at a time
            // -----------------------------------------------------------------------------------

            if (!gamepad2.left_bumper && leftBumperWasPressed){
                leftBumperWasPressed = false;
            }
            if (cone < 0 ) {  //checks to see if the arm is already at zero and doesn't try to move down anymore
                cone = 1;
            } else if (gamepad2.left_bumper && !leftBumperWasPressed) {
                cone = cone - 60;
                leftSlide.setTargetPosition(cone);  //stack incrementally move arm down by 50 on each press
                rightSlide.setTargetPosition(cone );
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftBumperWasPressed = true;
            } else if (gamepad2.right_bumper) {
                cone = 300;
                leftSlide.setTargetPosition(cone);  //stack height for 5 cones
                rightSlide.setTargetPosition(cone);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }

            if ((gamepad1.right_stick_button || gamepad2.right_stick_button) && !hasReset){
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(-0.5);
                rightSlide.setPower(-0.5);
                hasReset = true;
            } else if ((!gamepad1.right_stick_button || !gamepad2.right_stick_button)  && hasReset) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hasReset = false;
            }

            if ((!leftSlide.isBusy() || !rightSlide.isBusy()) && !hasReset) {
                //stop the slide and keep it from holding position
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0.05);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setPower(0.05);
            }

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

            // Telemetry Code

            // Show the elapsed run time
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // Show the power of the drive motors
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFrontPower, drive.rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBackPower, drive.rightBackPower);

            // Show the position of the arm
            telemetry.addData("ArmHeightL: ", leftSlide.getCurrentPosition());
            telemetry.addData("ArmHeightR: ", rightSlide.getCurrentPosition());
            telemetry.addData("Cone: ", cone);

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

