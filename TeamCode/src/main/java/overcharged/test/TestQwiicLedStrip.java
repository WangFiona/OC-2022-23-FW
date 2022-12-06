package overcharged.test;

//import android.graphics.Color;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import overcharged.components.QwiicLEDStrip;

@Disabled
@TeleOp(name = "LEDStripTest", group = "test")
public class TestQwiicLedStrip extends OpMode {
    private QwiicLEDStrip ledStrip;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private int colorIndex = 0;
    private int[] colors = new int[] { Color.rgb(148, 0, 211), Color.rgb(75, 0, 130), Color.rgb(0, 0, 255),
            Color.rgb(0, 255, 0), Color.rgb(255, 0, 0), Color.rgb(255, 255, 0), Color.parseColor("purple"),
            Color.parseColor("teal"), Color.parseColor("silver"), Color.rgb(0, 0, 0) };

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        ledStrip = hardwareMap.get(QwiicLEDStrip.class, "ledstrip");
        ledStrip.setBrightness(5); //max 16
        ledStrip.setColor(Color.RED);
        ledStrip.setColor(5, Color.BLUE);
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (elapsedTime.milliseconds() >= 500) {
            if (colorIndex == colors.length) {
                ledStrip.setColors(colors);
                colorIndex = 0;
            } else {
                ledStrip.setColor(colors[colorIndex]);
                colorIndex++;
            }
            elapsedTime.reset();
        }
    }

    @Override
    public void stop() {
        ledStrip.turnAllOff();
    }
}
