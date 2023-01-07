package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ledLights {
    public final RevBlinkinLedDriver lights;
    public ledStates currentState;

    public enum ledStates {
        INIT,
        OPEN,
        RED,
        BLUE,
        CLOSE,
    }

    public ledLights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        currentState = ledStates.INIT;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void setLed(ledStates state) {
        switch (state) {
            case OPEN:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                currentState = state;
                break;
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                currentState = state;
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                currentState = state;
                break;
            case CLOSE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
                currentState = state;
                break;
        }
    }
}
