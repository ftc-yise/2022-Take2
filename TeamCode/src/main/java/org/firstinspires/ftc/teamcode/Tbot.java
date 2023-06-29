package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;



@TeleOp(name="TBot" +
        "", group="Linear Opmode")
public class Tbot extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo sholder = null;
    private Servo elbow = null;

    public float speedmulti = 1;
    public float SholderValue = 0;
    public float ElbowValue = 0;

    //private Servo fireServo = null;
    //private CRServo barrel = null;
    TouchSensor limit;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        sholder = hardwareMap.get(Servo.class, "Shoulder");
        elbow = hardwareMap.get(Servo.class, "Elbow");
        //fireServo = hardwareMap.get(Servo.class, "fire_servo");
        //barrel = hardwareMap.get(CRServo.class, "barrel" );

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //fireServo.setDirection(Servo.Direction.FORWARD);
        sholder.setDirection(Servo.Direction.FORWARD);
        elbow.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        /*if (limit.isPressed()) {
            leftFrontDrive.setPower(0);
        } else { // Otherwise, run the motor
            leftFrontDrive.setPower(1);
        }*/


        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Updated upstream
            //Manual slide

            TouchSensor limit = null;

            /*if (limit.isPressed()) {
                leftBackDrive.setPower(0);
            } else { // Otherwise, run the motor
                leftBackDrive.setPower(1);
            }*/

            if(gamepad1.y && speedmulti == 1){
                speedmulti = .5f;
            } else if(gamepad1.y && speedmulti == .5f){
                speedmulti = 1f;
            }

            telemetry.addData("motor Power:", leftBackDrive.getPower());


            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
            double horizontal =  gamepad1.left_stick_y;
            double turn     =  gamepad1.left_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = /*vertical*/  horizontal - turn * speedmulti;
            double rightFrontPower = /*vertical*/  horizontal + turn * speedmulti;
            double leftBackPower   = /*vertical*/  horizontal - turn * speedmulti;
            double rightBackPower  = /*vertical*/  horizontal + turn * speedmulti;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            if(gamepad1.right_stick_x > .5){
                leftFrontPower = -1 * speedmulti;
                leftBackPower = -1 * speedmulti;
                rightBackPower = 1 * speedmulti;
                rightFrontPower = 1 * speedmulti;
            }else if(gamepad1.right_stick_x  < -.5){
                leftFrontPower = 1 * speedmulti;
                leftBackPower = 1 * speedmulti;
                rightBackPower = -1 * speedmulti;
                rightFrontPower = -1 * speedmulti;
            }
            if(gamepad1.left_stick_x < -.5){
                leftFrontPower = 1 * speedmulti;
                leftBackPower = -1 * speedmulti;
                rightBackPower = -1 * speedmulti;
                rightFrontPower = 1 * speedmulti;
            }else if(gamepad1.left_stick_x  > .5){
                leftFrontPower = -1 * speedmulti;
                leftBackPower = 1 * speedmulti;
                rightBackPower = 1 * speedmulti;
                rightFrontPower = -1 * speedmulti;
            }
            if(gamepad1.right_bumper) {
                sholder.setPosition(-.6);
                elbow.setPosition(-.6);
            }else if(gamepad1.left_bumper){
                sholder.setPosition(Servo.MAX_POSITION);
                elbow.setPosition(Servo.MAX_POSITION);
            }else if(gamepad1.right_trigger > .5) {
                sholder.setPosition(SholderValue);
                elbow.setPosition(ElbowValue);
                SholderValue = (float) (SholderValue + .01);
                ElbowValue = (float) (ElbowValue + .01);
            }else if(gamepad1.left_trigger > .5) {
                sholder.setPosition(SholderValue);
                elbow.setPosition(ElbowValue);
                ElbowValue = (float) (ElbowValue - .01);
                SholderValue = (float) (SholderValue - .01);
            }if(gamepad1.dpad_up) {
                sholder.setPosition(SholderValue);
                SholderValue = (float) (SholderValue + .01);
            }else if(gamepad1.dpad_left) {
                elbow.setPosition(ElbowValue);
                ElbowValue = (float) (ElbowValue + .01);
            }if(gamepad1.dpad_down) {
                sholder.setPosition(SholderValue);
                SholderValue = (float) (SholderValue - .01);
            }else if(gamepad1.dpad_right) {
                elbow.setPosition(ElbowValue);
                ElbowValue = (float) (ElbowValue - .01);
            }else if(!gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && gamepad1.right_trigger < .5 && gamepad1.left_trigger < .5) {
                elbow.setPosition(ElbowValue);
                sholder.setPosition(SholderValue);
                ElbowValue = (float) (ElbowValue - .0001);
                SholderValue = (float) (SholderValue + .000001);
            }



            /*else if(gamepad1.right_trigger > .5){
                sholder.setD;
                elbow.setPosition();
            }*/
            // testing out how to make the motors stop on a dime by keeping constant power so they dont spin freely
           /* if(gamepad1.left_stick_x == 0){
                leftFrontPower = .01;
                leftBackPower = -.01;
                rightBackPower = -.01;
                rightFrontPower = .01;
            }else if(gamepad1.right_stick_x == 0) {
                leftFrontPower = -.01;
                leftBackPower = .01;
                rightBackPower = .01;
                rightFrontPower = -.01;
            }else if(gamepad1.right_stick_y == 0) {
                leftFrontPower = -.01;
                leftBackPower = .01;
                rightBackPower = .01;
                rightFrontPower = -.01;
            }else if(gamepad1.left_stick_y == 0) {
                leftFrontPower = -.01;
                leftBackPower = .01;
                rightBackPower = .01;
                rightFrontPower = -.01;
            }*/
            /*if (gamepad1.a){
                fireServo.setPosition(Servo.MAX_POSITION);
            }
            else if (gamepad1.b){
                fireServo.setPosition(Servo.MIN_POSITION);

            }

            if (gamepad1.left_bumper){
                barrel.setPower(1);

            }else if (gamepad1.right_bumper){
                barrel.setPower(-1);

            }
            else
                barrel.setPower(0);








            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("limitSwitchState", limitSwitchState);


            //telemetry.addData("CR positon",fireServo.getPosition());
            //telemetry.addData("CR class",fireServo.getClass());
            telemetry.update();
        }
    }}


