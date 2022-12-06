package overcharged.components;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Parthiv on 2/20/2020.
 * This class uses both the rev hub's imu to get a better reading
 */
public abstract class OcGyro2
{
    protected volatile float baseHeadingL = 0;
    protected volatile float baseHeadingR = 0;

    protected String idL;
    protected String idR;

    protected OcGyro2(String l, String r) {
        this.idL = l;
        this.idR = r;
    }

    public String getId() {
        return idL + " " + idR;
    }

    public String toString () {
        return idL + " " + idR;
    }

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    public float getHeading() {
        float headingL = (getRawLHeading() - baseHeadingL + 360f) % 360f;
        if (headingL > 180f) {
            headingL = headingL - 360f;
        }
        float headingR = (getRawRHeading() - baseHeadingR + 360f) % 360f;
        if (headingR > 180f) {
            headingR = headingR - 360f;
        }
        return ((headingL + headingR) / 2);
    }

    public float getLHeading() {
        float headingL = (getRawLHeading() - baseHeadingL + 360f) % 360f;
        if (headingL > 180f) {
            headingL = headingL - 360f;
        }
        return headingL;
    }

    public float getRHeading() {
        float headingR = (getRawRHeading() - baseHeadingR + 360f) % 360f;
        if (headingR > 180f) {
            headingR = headingR - 360f;
        }
        return headingR;
    }

    public abstract float getRawLHeading();
    public abstract float getRawRHeading();

    /**
     * reset gyro headings to the current gyro reading
     */
    public void resetHeading() {
        baseHeadingL = getRawLHeading();
        baseHeadingR = getRawRHeading();
    }

    public void resetHeading(float headingL, float headingR){
        headingL = headingL % 360f;
        if (headingL > 180) {
            headingL = headingL - 360;
        }
        this.baseHeadingL = headingL;
        headingR = headingR % 360f;
        if (headingR > 180) {
            headingR = headingR - 360;
        }
        this.baseHeadingR = headingR;
    }

    public void adjustHeading(float adjust) {
        float bhL = (this.baseHeadingL + adjust) % 360f;
        if (bhL > 180) {
            bhL = bhL - 360;
        }
        this.baseHeadingL = bhL;
        float bhR = (this.baseHeadingR + adjust) % 360f;
        if (bhR > 180) {
            bhR = bhR - 360;
        }
        this.baseHeadingR = bhR;
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

    public double getFirstAngle() {
        Orientation anglesL = getLAngularOrientation();
        Orientation anglesR = getRAngularOrientation();
        return ((anglesL.firstAngle + anglesR.firstAngle) / 2);
    }

    public double getLFirstAngle() {
        return getLAngularOrientation().firstAngle;
    }

    public double getRFirstAngle() {
        return getRAngularOrientation().firstAngle;
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
     * Returns the calibration status of the left side IMU
     * @return the calibration status of the IMU
     */
    public abstract BNO055IMU.CalibrationStatus getLCalibrationStatus();

    /**
     * Returns the calibration status of the right side IMU
     * @return the calibration status of the IMU
     */
    public abstract BNO055IMU.CalibrationStatus getRCalibrationStatus();

    /**
     * get the rotated stance in three-dimensional space of the left IMU
     * by way of a set of three successive rotations.
     * @return Orientation
     */
    public abstract Orientation getLAngularOrientation();

    /**
     * get the rotated stance in three-dimensional space of the right IMU
     * by way of a set of three successive rotations.
     * @return Orientation
     */
    public abstract Orientation getRAngularOrientation();
}
