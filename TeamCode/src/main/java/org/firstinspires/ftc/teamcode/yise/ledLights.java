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
        BADHOVER,
        HOVER,
    }

    public ledLights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        currentState = ledStates.INIT;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
    }

    public void setLed(ledStates state) {
        switch (state) {
            case OPEN:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                currentState = state;
                break;
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                currentState = state;
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                currentState = state;
                break;
            case CLOSE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                currentState = state;
                break;
            case BADHOVER:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                currentState = state;
                break;
            case HOVER:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                currentState = state;
                break;
        }
    }
}
