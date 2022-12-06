package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import static overcharged.config.RobotConstants.TAG_H;

/**
 * Created by Kevin on 11/22/2017.
 * 11/14/2018 Updated with stall protection, Advaith Nair's idea
 * 1/9/2020 Updated with scaling and protection, Parthiv Nair's idea
 */
public class OcServo
        extends OcDevice {

    private Servo servo;
    float initialPosition;

    /**
     * initialize servos
     * @param hardwareMap HardwareMap to get servos from
     * @param id name of servo
     * @param initialPosition starting servo position
     * @throws IllegalArgumentException exception
     */
    public OcServo(HardwareMap hardwareMap,
                   String id,
                   float initialPosition)
            throws IllegalArgumentException
    {
        super(id);
        RobotLog.ii(TAG_H, "Constructor for Servo " + id);
        this.initialPosition = initialPosition;

        this.servo = hardwareMap.servo.get(id);
        setPosition(initialPosition);
    }

    /**
     * initialize servos
     * @param servo servo
     * @param id name of servo
     * @param initialPosition starting servo position
     * @throws IllegalArgumentException exception
     */
    public OcServo(Servo servo,
                   String id,
                   float initialPosition)
            throws IllegalArgumentException
    {
        super(id);
        RobotLog.ii(TAG_H, "Constructor for Servo " + id);
        this.initialPosition = initialPosition;

        this.servo = servo;
        setPosition(initialPosition);
    }

    /**
     * get servo
     * @return current servo
     */
    public Servo getServo() {
        return servo;
    }

    /**
     * get servo position
     * @return current servo position
     */
    public float getPosition() {
        return (float)(servo.getPosition() * 255f);
    }

    /// record the last position the servo was set to
    private float lastPosition = -1;

    /**
     * set servo position
     * @param position position to set servo to
     */
    public void setPosition(float position) {
        /// only change the servo position if it has changed
        if(Math.abs(lastPosition - position) > 0.5f){
            RobotLog.ii(TAG_H, "setPosition for Servo %s f power %f", this.id, position);
            servo.setPosition(position/255f);
            lastPosition = position;
        }
    }

    /**
     * get servo initial position
     * @return initial servo position
     */
    public float getInitialPosition() {
        return initialPosition;
    }

    /**
     * set servo to initial position
     */
    public void setInitialPosition() {
        setPosition(initialPosition);
    }
}