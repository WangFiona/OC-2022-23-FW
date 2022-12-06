package overcharged.linear.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.OcDevice;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static overcharged.config.RobotConstants.TAG_H;

public class OcEncoder extends OcDevice {

    private DcMotorEx encoder;
    private int encoderBase = 0;
    private static final double CIRCUMFERENCE = 1.9685 * Math.PI; //in
    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    //public static final double COUNTS_PER_INCH = 235f; // 360 / CIRCUMFERENCE;
    public static final double COUNTS_PER_INCH = 9.55592804147588;
    /**
     * initialize the encoder
     * @param hardwareMap HardwareMap to get motor encoder from
     * @param id name of motor
     * @param direction direction of motor
     */
    public OcEncoder(HardwareMap hardwareMap,
                     String id,
                     DcMotorEx.Direction direction)
            throws IllegalArgumentException
    {
        super(id);
        RobotLog.ii(TAG_H, "Constructor for OcEncoder " + id);
        this.encoder = (DcMotorEx)hardwareMap.get(DcMotor.class, id);
        this.encoder.setDirection(direction);
        encoder.setTargetPosition(0);
        this.encoder.setMode(RUN_USING_ENCODER);
    }

    /**
     * reset the encoder value to 0
     */
    public void reset() {
        resetPosition(0);
    }

    /**
     * update the encoder base to reset the encoder
     */
    private void resetPosition(int position) {
        encoderBase = encoder.getCurrentPosition() - position;
    }

    /**
     * get the encoder position without considering the last reset
     * @return encoder position
     */
    public double getPosition(){
        return encoder.getCurrentPosition();
    }

    /**
     * get the encoder target without considering the last reset
     * @return encoder target
     */
    public int getTarget() {
        return encoder.getTargetPosition();
    }

    /**
     * get the current encoder position
     * @return current encoder position
     */
    public double getCurrentPosition(){
        return encoder.getCurrentPosition() - encoderBase;
    }

    /**
     * get the encoder target position
     * @return encoder target position
     */
    public int getTargetPosition() {
        return encoder.getTargetPosition() - encoderBase;
    }

    /**
     * get the encoder inches moved
     * @return current encoder target position
     */
    public double getDistance(){
        return getCurrentPosition() / COUNTS_PER_INCH;
    }
}
