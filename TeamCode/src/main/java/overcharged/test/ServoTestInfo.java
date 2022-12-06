package overcharged.test;

import overcharged.components.OcServo;

/**
 * Created by Parthiv on 10/30/2017.
 */

public class ServoTestInfo {
    public OcServo servo;
    public int timeScale = 1;
    public float[] positions;

    /**
     * @param servo
     * @param positions servo positions other than the initial position
     */
    public ServoTestInfo(OcServo servo,
                         float... positions) {
        this.servo = servo;
        this.positions = positions;
    }

    /**
     * @param servo
     * @param positions servo positions other than the initial position
     */
    public ServoTestInfo(OcServo servo,
                         int timeScale,
                         float... positions) {
        this(servo, positions);
        this.timeScale = timeScale;
    }
}
