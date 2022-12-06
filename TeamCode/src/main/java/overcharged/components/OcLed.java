package overcharged.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static overcharged.components.OcLed.ELedStatus.LED_BLINK;
import static overcharged.components.OcLed.ELedStatus.LED_OFF;
import static overcharged.components.OcLed.ELedStatus.LED_ON;

/**
 * Created by Parthiv Nair 10/31/2020.
 */
public class OcLed extends OcDevice {

    private DigitalChannel channel;

    private ELedStatus status = LED_OFF;

    private ELedStatus pstatus = status;

    private long blinkStartTime = System.currentTimeMillis();

    private boolean isBlinkOn = true;

    public enum ELedStatus
    {
        LED_ON,
        LED_OFF,
        LED_BLINK
    }

    private static final int BLINK_DURATION = 200;

    /**
     * initialize the LEDs
     * @param hardwareMap HardwareMap to get motor from
     * @param ledName name of switches
     */
    public OcLed(HardwareMap hardwareMap,
                 String ledName) {
        super(ledName);
        // get a reference to our LED object.
        channel = hardwareMap.get(DigitalChannel.class, ledName);

        // set the channel channel to input.
        channel.setMode(DigitalChannel.Mode.OUTPUT);
        channel.setState(false);
    }

    /**
     * update the LED status
     */
    public void draw () {
        if (this.status == this.pstatus && this.status != LED_BLINK) return;
        switch (this.status) {
            case LED_BLINK: {
                channel.setState(isBlinkOn);
                long timeStamp = System.currentTimeMillis();
                if (timeStamp - blinkStartTime > BLINK_DURATION) {
                    blinkStartTime = timeStamp;
                    isBlinkOn = !isBlinkOn;
                }
                break;
            }
            case LED_ON:
                channel.setState(true);
                break;
            case LED_OFF:
            default:
                channel.setState(false);
                break;
        }
        this.pstatus = this.status;
    }

    /**
     * set LED on
     */
    public void on() {
        this.status = LED_ON;
    }

    /**
     * set LED off
     */
    public void off() {
        this.status = LED_OFF;
    }

    /**
     * set LED blink
     */
    public void blink() {
        this.status = LED_BLINK;
    }

    /**
     * set LED on
     * @param on if LED is on
     */
    public void on(boolean on) {
        set(on);
    }

    /**
     * set LED on
     * @param on if LED is on
     */
    public void set(boolean on) {
        this.status = on ? LED_ON : LED_OFF;
    }

    /**
     * set LED status
     * @param status if LED is status
     */
    public void set(ELedStatus status) {
        this.status = status;
    }

    /**
     * get LED on
     * @return if LED is on
     */
    public boolean isOn () {
        return this.status == LED_ON;
    }

    /**
     * get LED status
     * @param status the given status
     * @return if LED is the given status
     */
    public boolean isStatus (ELedStatus status) {
        return this.status == status;
    }
}
