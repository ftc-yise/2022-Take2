package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class liftArm {
    public final DcMotor leftSlide, rightSlide;

    // Used to set pole height.
    public enum Heights {
        LOW,
        MEDIUM,
        HIGH,
        HOVER
    }

    public liftArm(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

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

    public void returnToBottom() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public boolean forceDown() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(-0.5);
        rightSlide.setPower(-0.5);
        return false;
    }

    public boolean stopAndReset() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
    }

    public void holdPosition() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(0.05);
        rightSlide.setPower(0.05);

    }
}