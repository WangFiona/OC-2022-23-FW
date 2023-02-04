package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorEx.ZeroPowerBehavior;
import static overcharged.config.RobotConstants.TAG_H;

/**
 * Created by Parthiv on 1/30/2020.
 * 11/14/2018 Updated with stall protection, Advaith Nair's idea
 * 1/9/2020 Updated with scaling and protection, Parthiv Nair's idea

 */
public class OcMotorEx
        extends OcDevice {

    private DcMotorEx motor;
    public int encoderBase = 0;

    /**
     * initialize the motor
     * @param hardwareMap HardwareMap to get motor from
     * @param id name of motor
     * @param direction direction of motor
     */
    public OcMotorEx(HardwareMap hardwareMap,
                     String id,
                     DcMotorEx.Direction direction)
            throws IllegalArgumentException
    {
        this(hardwareMap,
                id,
                direction,
                DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * initialize the motor
     * @param hardwareMap HardwareMap to get motor from
     * @param id name of motor
     * @param direction direction of motor
     * @param runMode runMode of motor
     */
    public OcMotorEx(HardwareMap hardwareMap,
                     String id,
                     DcMotorEx.Direction direction,
                     DcMotorEx.RunMode runMode)
            throws IllegalArgumentException
    {
        super(id);
        RobotLog.ii(TAG_H, "Constructor for Motor " + id);
        this.motor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, id); //(DcMotorEx) hardwareMap.DcMotorEx.get(id);
        this.motor.setDirection(direction);
        this.motor.setMode(runMode);
        this.motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    public DcMotorEx getMotor () {
        return motor;
    }

    /// record the last power the motor was set to
    private float lastPower = -1.0f;

    /**
     * set motor power
     * @param power power to set the motor to
     */
    public void setPower(float power) {
        /// only change the motor power if it has changed
        if(Math.abs(power - lastPower) > 0.005f ||
                (power == 0f && lastPower != 0f)){
            RobotLog.ii(TAG_H, "setPower for motor %s f power %f", this.id, power);
            motor.setPower(power);//set the power of the motor
            lastPower = power;
        }
    }


    /**
     * set motor power
     * @param power power to set the motor to
     */
    /*public void setPower(double power) {
        setPower((float) power);
    }*/

    /**
     * @return motor power
     */
    public float getPower() {
        return (float) motor.getPower();
    }

    /**
     * change coefficients using methods included with DcMotorEx class.
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public void setPIDFCoefficients(double p, double i, double d, double f)
    {
        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(p, i, d, f);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
    }

    public void setTargetPositionPIDFCoefficients(double p, double i, double d, double f)
    {
        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(p, i, d, f);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidNew);
    }

    PIDFCoefficients getPIDCoefficients(){
        return motor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    /**
     * set motor power to float
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    /**
     * @return if power float
     */
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    /**
     * @return if motor is busy
     */
    public boolean isBusy() {
        return motor.isBusy();
    }

    /**
     * @return motor encoder value
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() - encoderBase;
    }

    /**
     * reset the motor encoder value
     */
    public void resetPosition() {
        resetPosition(0);
    }

    /**
     * update the motor encoder base to reset the encoder
     */
    public void resetPosition(int position) {
        encoderBase = motor.getCurrentPosition() - position;
    }

    /**
     * set the motor runMode
     * @param runMode motor mode
     */
    public void setMode(DcMotorEx.RunMode runMode) {
        motor.setMode(runMode);
    }


    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    public double getVelocity(){
        return motor.getVelocity();
    }

    /**
     * Returns the current velocity of the motor, in angular units per second
     * @param unit          the units in which the angular rate is desired
     * @return              the current velocity of the motor
     *
     * @see #setVelocity(double, AngleUnit)
     */
    public double getVelocity(AngleUnit unit){
        return motor.getVelocity(unit);
    }

    /**
     * Sets the velocity of the motor
     * @param angularRate  the desired ticks per second
     */
    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }

    /**
     * Sets the velocity of the motor
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     * @see #getVelocity(AngleUnit)
     */
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    /**
     * A shorthand for setting the PIDF coefficients for the {@link DcMotorEx.RunMode#RUN_USING_ENCODER}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     *
     */
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    /**
     * get the motor run mode
     * @return run mode of motor
     */
    public DcMotorEx.RunMode getMode() {
        return motor.getMode();
    }

    /**
     * set the motor target position
     * @param position target position
     */
    public void setTargetPosition(int position) {
        motor.setTargetPosition(encoderBase + position);
    }

    /**
     * get the current encoder target position
     * @return current encoder target position
     */
    public int getTargetPosition() {
        return motor.getTargetPosition() - encoderBase;
    }

    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }
}
