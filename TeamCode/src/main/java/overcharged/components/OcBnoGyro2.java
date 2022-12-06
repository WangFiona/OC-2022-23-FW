package overcharged.components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Parthiv Nair on 2/20/2020
 * This class uses both the rev hub's imu to get a better reading
 */

public class OcBnoGyro2
        extends OcGyro2 {

    public BNO055IMU gyroL;
    public BNO055IMU gyroR;

    /**
     * initialize the gyro sensor
     * @param hardwareMap hardware map
     */
    public OcBnoGyro2(HardwareMap hardwareMap,
                      String gyroLName, String gyroRName) {
        super(gyroLName, gyroRName);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyroL = hardwareMap.get(BNO055IMU.class, gyroLName);
        gyroR = hardwareMap.get(BNO055IMU.class, gyroRName);
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
        gyroL.initialize(parameters);
        gyroR.initialize(parameters);
        remap(gyroL);
        remap(gyroR);
    }

    public void remap(BNO055IMU gyro) {
        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0b00011000; //swaps y-z, 0b00100001 is y-x, 0x6 is x-z
            byte AXIS_MAP_SIGN_BYTE = 0b001; //x, y, z
            //Need to be in CONFIG mode to write to registers
            gyro.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
            Thread.sleep(100); //Changing modes requires a delay before doing anything else
            //Write to the AXIS_MAP_CONFIG register
            gyro.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
            //Write to the AXIS_MAP_SIGN register
            gyro.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
            //Need to change back into the IMU mode to use the gyro
            gyro.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public float getRawLHeading() {
        Orientation anglesL = gyroL.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return anglesL.firstAngle;
    }
    @Override
    public float getRawRHeading() {
        Orientation anglesR = gyroR.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return anglesR.firstAngle;
    }

    @Override
    public boolean isCalibrating() {
        return !gyroL.isGyroCalibrated() || !gyroR.isGyroCalibrated();
    }

    @Override
    public void calibrate() {

    }

    @Override
    public BNO055IMU.CalibrationStatus getLCalibrationStatus() {
        return gyroL.getCalibrationStatus();
    }

    @Override
    public BNO055IMU.CalibrationStatus getRCalibrationStatus() {
        return gyroR.getCalibrationStatus();
    }

    @Override
    public Orientation getLAngularOrientation() {
        return gyroL.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public Orientation getRAngularOrientation() {
        return gyroR.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /// Start the logging of measured acceleration
    public void startAccelerationIntegration() {
        ((BNO055IMU)gyroL).startAccelerationIntegration(new Position(), new Velocity(), 1000);
        ((BNO055IMU)gyroR).startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
}
