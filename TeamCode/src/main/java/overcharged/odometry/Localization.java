package overcharged.odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import overcharged.components.OcGyro;
import overcharged.components.OcGyro2;
import overcharged.linear.components.OcEncoder;

import static overcharged.config.RobotConstants.TAG_O;

/**
 * Created by Advaith Nair on 3/1/2020.
 */
public class Localization
{
    private OcEncoder encoderL;
    private OcEncoder encoderR;
    private OcGyro2 gyroSensor;
    //Orientation lastAngles = new Orientation();
    double lastAngle = 0;
    double globalAngle;

    private double dX;
    private double dTheta;

    private double x;
    private double y;
    private double theta;

    private double xLeftPrev;
    private double xRightPrev;
    private double thetaPrev;

    /**
     * Default Constructor (For Testing)
     */
    public Localization(HardwareMap hardwareMap, String leftEncoderId, String rightEncoderId, OcGyro2 gyro) {
        this(hardwareMap, leftEncoderId, rightEncoderId, gyro, 0, 0, 0);
    }

    /**
     * Preset Constructor (For Auto)
     *
     * @param hardwareMap
     * @param leftEncoderId
     * @param rightEncoderId
     * @param gyro
     * @param xCoord Starting X Coordinate
     * @param yCoord Starting Y Coordinate
     * @param angle Starting Angle
     */
    public Localization(HardwareMap hardwareMap, String leftEncoderId, String rightEncoderId,
                                OcGyro2 gyro, double xCoord, double yCoord, double angle) {
        x = xCoord;
        y = yCoord;
        xLeftPrev = xRightPrev = 0;
        theta = thetaPrev = angle;
        encoderL = new OcEncoder(hardwareMap, leftEncoderId, DcMotorEx.Direction.REVERSE);
        encoderR = new OcEncoder(hardwareMap, rightEncoderId, DcMotorEx.Direction.FORWARD);
        encoderL.reset();
        encoderR.reset();
        this.gyroSensor = gyro;
        RobotLog.ii(TAG_O, "Constructor for OdometryLocalization Left=", encoderR.getPosition() + " Right=" + encoderR.getPosition());
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        double angle = gyroSensor.getFirstAngle();
        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngle = angle;
        RobotLog.ii(TAG_O, "getAngle() angles=" + angle + " deltaAngle=" + deltaAngle + " globalAngle=" + globalAngle);
        return globalAngle;
    }

    /**
     * reset the encoder value to 0
     */
    public void resetEncoder() {
        encoderL.reset();
        encoderR.reset();
    }

    /**
     * Updates dX
     *
     * @param leftEncoder   Left Encoder Reading (in Inches)
     * @param rightEncoder  Right Encoder Reaching (in Inches)
     */
    private void setdX(double leftEncoder, double rightEncoder) {
        double dXL = (leftEncoder - xLeftPrev);
        double dXR = (rightEncoder - xRightPrev);

        dX = dXL + ((dXR - dXL)/2);

        xLeftPrev = leftEncoder;
        xRightPrev = rightEncoder;
    }

    /**
     * Updates Theta
     *
     * @param imuAngle  IMU Angle Reading (in Degrees)
     */
    private void setdTheta(double imuAngle) {
        dTheta = imuAngle - thetaPrev;
        thetaPrev = imuAngle;
        theta += dTheta;
    }

    /**
     * Updates Everything
     *
     * @param leftEncoder   Left Encoder Reading (in Inches)
     * @param rightEncoder  Right Encoder Reading (in Inches)
     * @param imuAngle      IMU Angle Reading (in Degrees)
     */
    private void update(double leftEncoder, double rightEncoder, double imuAngle) {
        setdX(leftEncoder, rightEncoder);
        setdTheta(imuAngle);

        x += (dX * Math.cos(Math.toRadians(theta)));
        y += (dX * Math.sin(Math.toRadians(theta)));
    }

    /**
     * Updates Everything
     */
    public void update() {
        update(encoderL.getCurrentPosition(), encoderR.getCurrentPosition(), getAngle());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    /**
     * Updates Values on Screen for Testing
     */
    public void showValues() {
        RobotLog.ii(TAG_O, String.format("Current X: (%f)", x));
        RobotLog.ii(TAG_O, String.format("Current Y: (%f)", y));
        RobotLog.ii(TAG_O, String.format("Current Theta: (%f)", theta));
    }
}
