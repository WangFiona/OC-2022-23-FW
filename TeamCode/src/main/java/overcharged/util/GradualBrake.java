package overcharged.util;

import overcharged.components.OcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Interlace braking and floating for a gradual braking
 */

public class GradualBrake {
    private long startTime = System.currentTimeMillis();;
    private boolean isFloat = true;
    private OcMotor[] motors;

    private static final int BRAKE_DURATION = 25;
    private static final int FLOAT_DURATION = 25;

    public GradualBrake (OcMotor... motors) {
        this.motors = motors;
    }

    public void brake () {
        brake(1f);
    }

    public void brake (float floatFactor) {
        if (motors == null || motors.length == 0) {
            return;
        }

        // float duration is adjusted according to float factor
        long timeStamp = System.currentTimeMillis();
        if ((isFloat && timeStamp - startTime > FLOAT_DURATION*floatFactor) ||
                (!isFloat && timeStamp - startTime > BRAKE_DURATION)){
            startTime = timeStamp;
            isFloat = !isFloat;
        }

        for (OcMotor motor: motors) {
            motor.setZeroPowerBehavior(isFloat? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }
}
