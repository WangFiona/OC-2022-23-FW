package overcharged.test;

import static overcharged.components.Button.BTN_NEXT;
import static overcharged.components.Button.BTN_PREV;
import static overcharged.config.RobotConstants.TAG_R;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.components.Button;
import overcharged.components.OcLed;

/**
 * Created by Parthiv Nair on 12/1/2019.
 * You can set the initial position using this program. This is useful especially when replacing servos
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestLed", group = "Test")
public class TestOcLed extends OpMode {
    ///Led indicator components
    private OcLed ledGreen;
    private OcLed ledYellow;
    private OcLed ledWhite;
    private OcLed ledBlue;
    private OcLed ledRed;
    public final List<OcLed> leds = new ArrayList<>();
    private ElapsedTime runtime = new ElapsedTime();


    /**
     * LED state
     */
    public enum LedState {
        ///Green
        GREEN,
        ///Yellow
        YELLOW,
        ///White
        WHITE,
        ///Blue
        BLUE,
        ///Red
        RED
    }
    LedState ledState = LedState.GREEN;

    @Override
    public void init() {
        hardwareMap.logDevices();
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        ///Initialize Leds
        OcLed led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_yellow");
            leds.add(led);
        } catch (Exception e) {
            missing = missing + ", led_yellow";
            numberMissing++;
        }
        ledYellow = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_green");
            leds.add(led);
        } catch (Exception e) {
            missing = missing + ", led_green";
            numberMissing++;
        }
        ledGreen = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_white");
            leds.add(led);
        } catch (Exception e) {
            missing = missing + ", led_white";
            numberMissing++;
        }
        ledWhite = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_blue");
            leds.add(led);
        } catch (Exception e) {
            missing = missing + ", led_blue";
            numberMissing++;
        }
        ledBlue = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_red");
            leds.add(led);
        } catch (Exception e) {
            missing = missing + ", led_red";
            numberMissing++;
        }
        ledRed = led;
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.addData("Missing", missing);
        telemetry.update();
    }

    /**
     * update LEDs
     */
    public void drawLed () {
        for (OcLed led: leds) {
            led.draw();
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void turnYellowOn(boolean on) {
        try {
            if (on) {
                this.ledYellow.on();
            } else {
                this.ledYellow.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_yellow " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void turnBlueOn(boolean on) {
        try {
            if (on) {
                this.ledBlue.on();
            } else {
                this.ledBlue.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Blue " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void turnGreenOn(boolean on) {
        try {
            if (on) {
                this.ledGreen.on();
            } else {
                this.ledGreen.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Green " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void blinkingGreen() {
        try {
            this.ledGreen.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Green " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void blinkingBlue() {
        try {
            this.ledBlue.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Blue " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void blinkingWhite() {
        try {
            this.ledWhite.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_White " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void blinkingRed() {
        try {
            this.ledRed.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Red " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void blinkingYellow() {
        try {
            this.ledYellow.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Yellow " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void turnRedOn(boolean on) {
        try {
            if (on) {
                this.ledRed.on();
            } else {
                this.ledRed.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Red " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void turnWhiteOn(boolean on) {
        try {
            if (on) {
                this.ledWhite.on();
            } else {
                this.ledWhite.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_White " + e.getMessage());
        }
    }
    private boolean isSlow = false;
    ///Initialize the Super Slow mode to false
    private boolean isSuperSlow = false;

    String status = "";
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Gamepad", "Left/Right Bumper, Left/Right Trigger");
        long timeStamp = System.currentTimeMillis();

        if (gamepad1.right_trigger > 0.9 && BTN_NEXT.canPress(timeStamp)) {
            if (ledState == LedState.GREEN) {
                ledState = LedState.YELLOW;
            } else if (ledState == LedState.YELLOW) {
                ledState = LedState.WHITE;
            } else if (ledState == LedState.WHITE) {
                ledState = LedState.BLUE;
            } else if (ledState == LedState.BLUE) {
                ledState = LedState.RED;
            } else if (ledState == LedState.RED) {
                ledState = LedState.GREEN;
            }
        } else if (gamepad1.left_trigger > 0.9 && BTN_PREV.canPress(timeStamp)) {
            if (ledState == LedState.GREEN) {
                ledState = LedState.RED;
            } else if (ledState == LedState.YELLOW) {
                ledState = LedState.GREEN;
            } else if (ledState == LedState.WHITE) {
                ledState = LedState.YELLOW;
            } else if (ledState == LedState.BLUE) {
                ledState = LedState.WHITE;
            } else if (ledState == LedState.RED) {
                ledState = LedState.BLUE;
            }
        }

        if (gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timeStamp)) {
            isSlow = !isSlow;
            isSuperSlow = false;
        } else if (gamepad1.left_bumper && Button.BTN_SUPERSLOW_MODE.canPress(timeStamp)) {
            isSuperSlow = !isSuperSlow;
            isSlow = false;
        }
        if (isSuperSlow) {
            if (ledState == LedState.GREEN) {
                status = "Green Blink";
                blinkingGreen();
            } else if (ledState == LedState.YELLOW) {
                status = "Yellow Blink";
                blinkingYellow();
            } else if (ledState == LedState.WHITE) {
                status = "White Blink";
                blinkingWhite();
            } else if (ledState == LedState.BLUE) {
                status = "Blue Blink";
                blinkingBlue();
            } else if (ledState == LedState.RED) {
                status = "Red Blink";
                blinkingRed();
            }
        } else if (isSlow) {
            if (ledState == LedState.GREEN) {
                status = "Green On";
                turnGreenOn(true);
            } else if (ledState == LedState.YELLOW) {
                status = "Yellow On";
                turnYellowOn(true);
            } else if (ledState == LedState.WHITE) {
                status = "White On";
                turnWhiteOn(true);
            } else if (ledState == LedState.BLUE) {
                status = "Blue On";
                turnBlueOn(true);
            } else if (ledState == LedState.RED) {
                status = "Red On";
                turnRedOn(true);
            }
        } else {
            if (ledState == LedState.GREEN) {
                status = "Green Off";
                turnGreenOn(false);
            } else if (ledState == LedState.YELLOW) {
                status = "Yellow Off";
                turnYellowOn(false);
            } else if (ledState == LedState.WHITE) {
                status = "White Off";
                turnWhiteOn(false);
            } else if (ledState == LedState.BLUE) {
                status = "Blue Off";
                turnBlueOn(false);
            } else if (ledState == LedState.RED) {
                status = "Red Off";
                turnRedOn(false);
            }
        }
        telemetry.addData("LED", status);
        telemetry.addData("Stop", "Back");
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }
        telemetry.update();
        drawLed();
    }
}