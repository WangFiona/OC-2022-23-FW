package overcharged.components;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static overcharged.config.RobotConstants.TAG_TDRV;

/**
 * Drive class for OpMode.
 */
public class TankDrive extends Drive
{

    public TankDrive(OcMotor driveLeftFront,
                     OcMotor driveLeftBack,
                     OcMotor driveRightFront,
                     OcMotor driveRightBack)
    {
        super(driveLeftFront,
              driveLeftBack,
              driveRightFront,
              driveRightBack);
    }

    /**
     * stop the drive motors
     */
    @Override
    public void stop() {
        setPower(0f);
    }

    /**
     * set drive motor powers for floats
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * run the drive motors
     * @param p motor power
     */
    public void setPower(double p) {
        driveLeftFront.setPower(p);
        driveLeftBack.setPower(p);
        driveRightFront.setPower(p);
        driveRightBack.setPower(p);
    }

    /**
     * turn the robot at a specified power
     * @param pwrl motor power left
     * @param pwrr motor power right
     */
    public void setPower(float pwrl,
                         float pwrr) {
        driveLeftFront.setPower(pwrl);
        driveLeftBack.setPower(pwrl);
        driveRightFront.setPower(pwrr);
        driveRightBack.setPower(pwrr);
    }

    /**
     * turn the robot at a specified power
     * @param pwrl motor power left
     * @param pwrr motor power right
     */
    public void setPower(double pwrl,
                         double pwrr) {
        driveRightFront.setPower(pwrr);
        driveRightBack.setPower(pwrr);
        driveLeftFront.setPower(pwrl);
        driveLeftBack.setPower(pwrl);
    }

    private float coordinatesToDegrees(float x, float y) {
        return 90 - Math.round(Math.toDegrees(Math.atan(Math.abs(y / x))));
    }

    private float coordinatesToDegrees(float x1, float y1, float x2, float y2) {
        float degrees = Math.round((coordinatesToDegrees(x1, y1) + coordinatesToDegrees(x2, y2))/2);
        //RobotLog.ii(TAG_TDRV, "coordinatesToDegrees x1=" + x1 + " y1=" + y1 + " x2=" + x2 + " y2=" + y2 + " degrees=" + degrees);
        /*if(degrees >= 60)
            return 60;*/
        return degrees;
    }

    ///Greater the value increases turn sensitivity
    private float TURN_SENSITIVITY = 0.2f;

    ///Calculate the power on the slower side
    private float slowPower(float turnAngle, float powerMult) {
        return Range.clip((powerMult * ((90f - turnAngle)/90f) - TURN_SENSITIVITY), 0, 1);
    }

    //int getYSign, return 1, -1, or 0
    //make sure it's not 0, then multiply current constants times getYSign value
    private int getY (float y1, float y2) {
        if(y1 > 0 && y2 > 0)
            return 1;
        else if (y1 < 0 && y2 < 0)
            return -1;
        return 0;
    }

    /**
     * Navigate using snake mode by using all quadrant x & y values
     * @param x1 gamepad left stick x value
     * @param y1 gamepad left stick y value
     * @param x2 gamepad right stick x value
     * @param y2 gamepad right stick y value
     * @param powerMult speed requested by the driver
     */
    public void setPower(float x1,
                         float y1, float x2,
                         float y2, float powerMult) {
        int ySign = getY(y1, y2);
        if(x1 < 0 && x2 < 0 && ySign != 0) {
            float calculatedPower = slowPower(coordinatesToDegrees(x1, y1, x2, y2), powerMult);
            //RobotLog.ii(TAG_TDRV, "setPower < 0 calculatedPower=" + calculatedPower);
            driveLeftFront.setPower(calculatedPower * ySign);
            driveLeftBack.setPower(calculatedPower * ySign);
            driveRightFront.setPower(powerMult * ySign);
            driveRightBack.setPower(powerMult * ySign);
        }
        else if (x1 > 0 && x2 > 0 && ySign != 0) {
            float calculatedPower = slowPower(coordinatesToDegrees(x1, y1, x2, y2), powerMult);
            //RobotLog.ii(TAG_TDRV, "setPower > 0 calculatedPower=" + calculatedPower);
            driveLeftFront.setPower(powerMult * ySign);
            driveLeftBack.setPower(powerMult * ySign);
            driveRightFront.setPower(calculatedPower * ySign);
            driveRightBack.setPower(calculatedPower * ySign);
        }
        else {
            //RobotLog.ii(TAG_TDRV, "setPower regular");
            driveLeftFront.setPower(y1 * powerMult);
            driveLeftBack.setPower(y1 * powerMult);
            driveRightFront.setPower(y2 * powerMult);
            driveRightBack.setPower(y2 * powerMult);
        }
    }

    /**
        * turn the robot at a specified power
        * @param pwrl motor power left
        * @param pwrr motor power right
        * @param turnType turning method to use
        */
    public void setPower(double pwrl,
                         double pwrr,
                         TurnType turnType) {
        switch (turnType) {
            case TURN_REGULAR:
                driveLeftFront.setPower(-pwrl);
                driveLeftBack.setPower(-pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case TURN_RIGHT_PIVOT:
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case TURN_LEFT_PIVOT:
                driveLeftFront.setPower(-pwrl);
                driveLeftBack.setPower(-pwrl);
                break;
            default:
                break;
        }
    }

    /**
     * get the 2nd largest encoder value of drives (2nd largest will most likely be accurate)
     * use bubble sort
     * @return the second largest encoder value of drives
     */
    public int getEncoder() {

        final int NUM_WHEELS = 4;
        // 2nd biggest
        final int RETURN_INDEX = 1;

        int enc[] = new int[NUM_WHEELS];
        enc[0] = driveLeftFront.getCurrentPosition();
        enc[1] = driveLeftBack.getCurrentPosition();
        enc[2] = driveRightFront.getCurrentPosition();
        enc[3] = driveRightBack.getCurrentPosition();

        // bubble sort
        for (int i = 0; i <= RETURN_INDEX; i++)
        {
            for (int j = i + 1; j < NUM_WHEELS; j++)
            {
                if (Math.abs(enc[i]) < Math.abs(enc[j]))
                {
                    // swap
                    int c = enc[i];
                    enc[i] = enc[j];
                    enc[j] = c;
                }
            }
        }

        return enc[RETURN_INDEX];
    }
    /*public double getEncoder() {

        double leftPosition = left.getPosition();
        double rightPosition = right.getPosition();

        return (leftPosition+rightPosition)/2;
    }*/
}
