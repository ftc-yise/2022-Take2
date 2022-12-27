/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.yise.mecanumDrive;


@TeleOp(name="Test AutoCentering", group="Linear Opmode")
public class TestDistanceSensor extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    public boolean canSwitchModes = false;

    @Override
    public void runOpMode() {

        mecanumDrive drive = new mecanumDrive(hardwareMap);

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

            // If we have any Dpad input, update the motor power based on Dpad (ie overright stick)
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }

            // Auto-Centering Code
            if (gamepad1.left_trigger >= 0.8) {
                drive.autoCenter();
            }

            // Show the elapsed game time and wheel power.
            // mecanumDrive.updateDistances();
            // telemetry.addData("Distance right: ", mecanumDrive.distanceLeft);
            // telemetry.addData("Distance left ", mecanumDrive.distanceRight);
            // telemetry.update();
        }
    }
}
