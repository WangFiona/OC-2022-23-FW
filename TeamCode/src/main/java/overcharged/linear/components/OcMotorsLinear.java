package overcharged.linear.components;

import overcharged.components.OcDevice;
import overcharged.components.OcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * Created by Kevin on 11/22/2017.
 */
public class OcMotorsLinear
    extends OcDevicesLinear
{
    public OcMotorsLinear(LinearOpMode op,
                          OcMotor... motors) {
        super(op, motors);
    }

    public OcMotorsLinear(OcMotor... motors) {
        super(motors);
    }

    /**
     * Stop motors until it is confirmed.
     * @throws InterruptedException
     */
    public void stop()
        throws InterruptedException
    {
        set(new Update() {
            public boolean isUpdated(OcDevice motor) {
                return Math.abs(((OcMotor) motor).getPower()) < 0.005;
            }

            public void update(OcDevice motor) {
                ((OcMotor) motor).setPower(0);
            }
        });
    }

    /**
     * set motor regulation until it is confirmed
     * @param mode motor regulation to set to
     */
    public void setMode (final RunMode mode)
        throws InterruptedException
    {
        set(new Update() {
            public boolean isUpdated(OcDevice motor) {
                return ((OcMotor) motor).getMode() == mode;
            }

            public void update(OcDevice motor) {
                ((OcMotor) motor).setMode(mode);
            }
        });
    }

    /**
     * set motor power float until it is confirmed.
     * @throws InterruptedException
     */
    public void setZeroPowerBehavior(final ZeroPowerBehavior behavior)
        throws InterruptedException
    {
        set(new Update() {
            public boolean isUpdated(OcDevice motor) {
                return ((OcMotor) motor).getZeroPowerBehavior() == behavior;
            }

            public void update(OcDevice motor) {
                ((OcMotor) motor).setZeroPowerBehavior(behavior);
            }
        });
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition(final int position)
            throws InterruptedException {
        setTargetPosition(position, null);
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition(final int position,
                                  final OcDevice[] motors)
            throws InterruptedException
    {
        set(new Update() {
                public boolean isUpdated(OcDevice motor) {
                    return Math.abs(((OcMotor) motor).getTargetPosition() - position) < 10;
                }

                public void update(OcDevice motor) {
                    ((OcMotor) motor).setTargetPosition(position);
                }
            },
            motors);
    }
}
