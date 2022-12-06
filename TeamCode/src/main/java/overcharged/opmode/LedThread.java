package overcharged.opmode;

import android.graphics.Color;

import overcharged.components.QwiicLEDStrip;

public class LedThread implements Runnable {

    public QwiicLEDStrip led;

    boolean[] states;

    private boolean ledYellowOn = false;
    private boolean ledRedOn = false;
    private boolean ledBlueOn = false;
    private boolean ledOff = false;
    private boolean ledWhiteOn = false;
    private boolean ledGreenOn = false;
    private boolean ledMagentaOn = false;
    private boolean ledCyanOn = false;

    public LedThread(QwiicLEDStrip led, boolean[] states) {
        this.led = led;
        this.states = states;
    }

    @Override
    public void run() {
        boolean running = states[0];

        while (running) {
            boolean isSlow = states[1];
            boolean isBlock = states[2];
            boolean isBall = states[3];
            boolean isProblem = states[4];

            if(isSlow && !ledRedOn){
                led.setColor(1, Color.RED);
                ledRedOn = true;
            } else if(!isSlow && ledRedOn){
                led.setColor(1, 0);
                ledRedOn = false;
            }

            if(isBlock && !ledYellowOn){
                led.setColor(2, Color.YELLOW);
                ledYellowOn = true;
            } else if(!isBlock && ledYellowOn){
                led.setColor(2, 0);
                ledYellowOn = false;
            }

            if(isBall && !ledWhiteOn){
                led.setColor(3, Color.WHITE);
                ledWhiteOn = true;
            } else if(!isBall && ledWhiteOn){
                led.setColor(3, 0);
                ledWhiteOn = false;
            }

            running = states[0];
        }
    }
}
