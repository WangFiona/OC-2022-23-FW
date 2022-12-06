package overcharged.linear.components;

import overcharged.components.OcDevice;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * This class perform an update multiple times until it is comfirmed.
 */
public class OcDevicesLinear
{
    protected OcDevice[] devices;

    protected LinearOpMode op;

    protected OcDevicesLinear(LinearOpMode op,
                              OcDevice... devices)
    {
        this.op = op;
        this.devices = devices;
    }

    protected OcDevicesLinear(OcDevice... devices)
    {
        this.devices = devices;
    }

    public void setLinearOpMode (LinearOpMode op) {
        this.op = op;
    }

    protected void set (Update update)
            throws InterruptedException {
        set(update,
            this.devices);
    }

    /**
     * This is used for update over multiple iterations. Sometimes
     * the update need to be done multiple times until
     * it is confirmed.
     * @param update update callback
     * @param devices devices to be updated
     * @throws InterruptedException
     */
    protected void set (Update update,
                        OcDevice[] devices)
            throws InterruptedException
    {
        if (devices == null || devices.length == 0) {
            devices = this.devices;
        }

        // init
        boolean[] isSets = new boolean[devices.length];
        Arrays.fill(isSets, false);

        boolean allSet;
        do {
            allSet = true;
            for (int i = 0; i < devices.length; i++) {
                if (!isSets[i]) {
                    if (update.isUpdated(devices[i])) {
                        isSets[i] = true;
                    } else {
                        update.update(devices[i]);
                    }
                }
                allSet &= isSets[i];
            }

            op.idle();
        }
        while (!allSet && op.opModeIsActive());
    }

    public interface Update {
        boolean isUpdated(OcDevice device);
        void update(OcDevice device);
    }
}
