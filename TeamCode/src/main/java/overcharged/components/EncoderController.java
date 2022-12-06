package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.linear.components.OcEncoder;

import static overcharged.config.RobotConstants.TAG_H;

public class EncoderController {
    public OcEncoder encoderL;
    public OcEncoder encoderR;

    public static final double CENTER_OFFSET = 6.125;
    public static final double ENCODER_GAP = 10.875;
    public static final double DISTANCE_FROM_CENTER = Math.sqrt(Math.pow(ENCODER_GAP / 2, 2)
            + Math.pow(CENTER_OFFSET, 2));

    public enum EncoderTurnType {
        CENTER,
        PIVOT_STRAIGHT
    }

    public EncoderController(HardwareMap hardwareMap,
                             String leftEncoderId, String rightEncoderId) {
        encoderL = new OcEncoder(hardwareMap, leftEncoderId, DcMotorEx.Direction.REVERSE);
        encoderR = new OcEncoder(hardwareMap, rightEncoderId, DcMotorEx.Direction.FORWARD);
        RobotLog.ii(TAG_H, "Constructor for EncoderController Left=", encoderR.getPosition() + " Right=" + encoderR.getPosition());
        encoderL.reset();
        encoderR.reset();
    }

    /**
     * reset the encoder value to 0
     */
    public void reset() {
        encoderL.reset();
        encoderR.reset();
    }

    public double getDistance() {
        return (encoderL.getDistance() + encoderR.getDistance())/2;
    }

    public String getDistanceToString() {
        return "Left=" + encoderL.getDistance() + " Right=" + encoderR.getDistance();
    }

    public double getCurrentPosition() {
        return (encoderL.getCurrentPosition() + encoderR.getCurrentPosition())/2;
    }

    public String getCurrentPositionString() {
        return "Left=" + encoderL.getCurrentPosition() + " Right=" + encoderR.getCurrentPosition();
    }

    public double getDrift() {
        if (encoderL.getDistance() == encoderR.getDistance()) return 0;
        return encoderR.getDistance() - encoderL.getDistance();
    }

    /**
     * Determining the number of degrees turned by the robot from the relative encoder values
     * @param turnType
     *     CENTER: Robot turns around the very center of it.
     *     PIVOT_STRAIGHT: Robot drives straight, pivots, or splines (in circular arc).
     *                     Center of rotational path is outside of the robot's body.
     * @param positionBase Starting values of the encoders, to find angle caused by the turn,
     *                     not the absolute angle/distance travelled by the robot.
     * @return Angle of the robot relative to the start of the turn.
     */
    public double getTurnDegrees(EncoderTurnType turnType, double positionBase) {
        double leftRelPos = encoderL.getCurrentPosition() - positionBase;
        double rightRelPos = encoderR.getCurrentPosition() - positionBase;

        if (turnType == EncoderTurnType.CENTER) {
            return ((leftRelPos + rightRelPos) / 2 / DISTANCE_FROM_CENTER * 180 / Math.PI);
        } else {
            /*
            Positive is angled left, implies
            negative is angled right, 0 is straight

             */

            return ((leftRelPos + rightRelPos) /
                    ((ENCODER_GAP * 2 * leftRelPos) / rightRelPos - leftRelPos) + ENCODER_GAP);
        }
    }
}
