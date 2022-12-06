package overcharged.components;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Kevin on 11/22/2017.
 */
public abstract class OcGyro
        extends OcDevice
{
    protected volatile float baseHeading = 0;

    protected OcGyro(String id) {
        super(id);
    }

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    public float getHeading() {
        float heading = (getRawHeading() - baseHeading + 360f) % 360f;
        if (heading > 180f) {
            heading = heading - 360f;
        }
        return heading;
    }

    public abstract float getRawHeading();

    /**
     * reset gyro headings to the current gyro reading
     */
    public void resetHeading() {
        baseHeading = getRawHeading();
    }

    public void resetHeading(float baseHeading){
        baseHeading = baseHeading % 360f;
        if (baseHeading > 180) {
            baseHeading = baseHeading - 360;
        }
        this.baseHeading = baseHeading;
    }

    public void adjustHeading(float adjust) {
        float bh = (this.baseHeading + adjust) % 360f;
        if (bh > 180) {
            bh = bh - 360;
        }
        this.baseHeading = bh;
    }

    /**
     * course adjust the direction
     * @return how much to adjust by
     */
    public float adjustDirection () {

        final float RATIO_SERVO_ADJUST = 30f;

        float deltaHeading = (360f - getHeading()) % 360f;
        if (deltaHeading > 180f) {
            deltaHeading = deltaHeading - 360f;
        }
        float adjust =  deltaHeading / RATIO_SERVO_ADJUST;

        //op.telemetry.addData("delta:", deltaHeading);
        //op.telemetry.addData("adjust:", adjust);

        return adjust;
    }

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    public abstract boolean isCalibrating();

    /**
     * calibrate the gyro sensor
     * @throws InterruptedException if the thread is interrupted
     */
    public abstract void calibrate() throws InterruptedException;

    /**
     * Returns the calibration status of the IMU
     * @return the calibration status of the IMU
     */
    public abstract BNO055IMU.CalibrationStatus getCalibrationStatus();

    /**
     * get the rotated stance in three-dimensional space
     * by way of a set of three successive rotations.
     * @return Orientation
     */
    public abstract Orientation getAngularOrientation();
}
