package overcharged.components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Kevin on 10/30/2017.
 * Updated for using PID by Parthiv Nair on 9/17/2019
 */

public class OcBnoGyro
        extends OcGyro {

    BNO055IMU gyro;

    /**
     * initialize the gyro sensor
     * @param hardwareMap hardware map
     */
    public OcBnoGyro(HardwareMap hardwareMap,
                     String gyroName) {
        super(gyroName);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, gyroName);
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.i2cAddr = I2cAddr.create7bit(0x28);
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //below 3 parameters were added to support Proportional Integral Derivative (PID)
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro.initialize(parameters);
    }

    @Override
    public float getRawHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    @Override
    public boolean isCalibrating() {
        return !gyro.isGyroCalibrated();
    }

    @Override
    public void calibrate() {

    }

    @Override
    public BNO055IMU.CalibrationStatus getCalibrationStatus() {
        return gyro.getCalibrationStatus();
    }

    @Override
    public Orientation getAngularOrientation() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /// Start the logging of measured acceleration
    public void startAccelerationIntegration() {
        ((BNO055IMU)gyro).startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
}
