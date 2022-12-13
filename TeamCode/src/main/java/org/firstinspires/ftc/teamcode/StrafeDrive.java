package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@TeleOp(name="Drive program", group="Linear Opmode")
public class StrafeDrive extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private static final String VUFORIA_KEY = "AYvN/Qn/////AAABmTaYCa1SUkR7v5u6+3uR8CkLP9DKgYYzF/zz/slKnKktetT5kD2UyolBbzcRUiOv5ve/Bo+wJ5V01NzDGkWOqbwCwdrrwIau2aTGiVuF+nclQtuaZrp5qnMiWm4G2FUpgmQgoIbKZF2jrtyKfA957eIT6TxfN6CmPuFCjq/hd9FUMg5NjBZKOB3MmsxkgclcDW1U7YfPdVSTZURhXrg705CVtRFtwwy5TPfWCn9fYPLCFihlQpmQrg1D/hvTmlWtqA2edw0u7LaEhzZtFjmgB/O98qlZPqFixSX0yOIpmY04434VJO7uF9+9NAjGnfgtX207yf2TwNm9S4mfe4hCuka1WdNUzdJUkxfciaBtn2BJ";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    WebcamName webcamName;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    //private Rev2mDistanceSensor distanceSensorRight = null;
    //private Rev2mDistanceSensor distanceSensorLeft = null;

    private Servo coneGrabber = null;

    public float speedMultiplier = 1;

    public int cone = 0;  // variable used to set the value for the cone heights and incremental down values

    boolean hasReset = true;

    public boolean canSwitchModes = true;

    public boolean leftBumperWasPressed = false;

    @Override
    public void runOpMode() {

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /*
        TouchSensor limit;
        String limitSwitchState;
        limit = hardwareMap.get(TouchSensor.class, "limit");
        if (limit.isPressed()) {
            limitSwitchState = "pressed";
        } else {
            limitSwitchState = "not_pressed";
        }
        */

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        coneGrabber = hardwareMap.get(Servo.class, "cone_grabber");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

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
            double max;
            //double distanceRight = distanceSensorRight.getDistance(DistanceUnit.CM);
            //double distanceLeft = distanceSensorLeft.getDistance(DistanceUnit.CM);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double vertical = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double horizontal = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = vertical + horizontal + turn;
            double rightFrontPower = vertical - horizontal - turn;
            double leftBackPower = vertical - horizontal + turn;
            double rightBackPower = vertical + horizontal - turn;

            /*leftSlide.setPower(-gamepad2.left_stick_y);
            rightSlide.setPower(-gamepad2.left_stick_y);*/

             /*
            ---------------------------------------------------------------------------------------
            TensorFlow Code
            ---------------------------------------------------------------------------------------
             */
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                    }
                    telemetry.update();
                }
            }

            /*
            ---------------------------------------------------------------------------------------
            3 Position Lift Arm Code
            ---------------------------------------------------------------------------------------
             */
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

            // Cone Stack Code - Go up to core 5 and step down 1 cone at a time
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

            // Normalize the values so no wheel power exceeds 100%
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

            if (gamepad1.a || gamepad2.a) {
                coneGrabber.setPosition(Servo.MIN_POSITION);
            } else if (gamepad1.b || gamepad2.b) {
                coneGrabber.setPosition(Servo.MAX_POSITION);
            }

            // Send calculated power to wheels
            if (gamepad1.y && (speedMultiplier == 1) && canSwitchModes) {
                speedMultiplier = 0.75f;
            } else if (gamepad1.y && (speedMultiplier == 0.75f) && canSwitchModes) {
                speedMultiplier = 1f;
            }

            if (gamepad1.y) {
                canSwitchModes = false;
            } else {
                canSwitchModes = true;
            }

            leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
            rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
            leftBackDrive.setPower(leftBackPower * speedMultiplier);
            rightBackDrive.setPower(rightBackPower * speedMultiplier);

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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("HeightL: ", leftSlide.getCurrentPosition());
            telemetry.addData("HeightR: ", rightSlide.getCurrentPosition());
            telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Rear Encoder:", rightFrontDrive.getCurrentPosition());
            telemetry.addData(" Right Encoder:", -rightBackDrive.getCurrentPosition());
            //telemetry.addData("Distance left: ", distanceLeft);
            //telemetry.addData("Distance right: ", distanceRight);
            // telemetry.addData("limitSwitchState", limitSwitchState);
            telemetry.addData("cone: ", cone);
            telemetry.update();

        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

}

