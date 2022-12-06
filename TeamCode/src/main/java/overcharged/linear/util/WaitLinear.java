package overcharged.linear.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class handle delay
 */
public class WaitLinear {
    private LinearOpMode op;

    public WaitLinear(LinearOpMode op) {
        this.op = op;
    }

    /**
     * Wait for the given milliseconds
     * @param millis milliseconds to wait
     * @throws InterruptedException
     */
    public void waitMillis (double millis)
        throws InterruptedException
    {
        long startTimestamp = System.currentTimeMillis();
        waitMillis(millis,
                   startTimestamp,
                   null);
    }

    /**
     * Wait for the given milliseconds
     * @param millis milliseconds to wait
     * @param startTimestamp start timestamp
     * @throws InterruptedException
     */
    public void waitMillis (int millis,
                            long startTimestamp)
        throws InterruptedException
    {
        waitMillis(millis,
                   startTimestamp,
                   null);
    }

    /**
     * Wait for the given milliseconds
     * @param millis milliseconds to wait
     * @param wakeup Callback for wakeup
     * @throws InterruptedException
     */
    public void waitMillis (int millis,
                            WakeUp wakeup)
        throws InterruptedException {
        long startTimestamp = System.currentTimeMillis();
        waitMillis(millis,
                   startTimestamp,
                   wakeup);
    }

        /**
        * Wait for the given milliseconds
        * @param millis milliseconds to wait
        * @param startTimestamp start timestamp
        * @param wakeup Callback for wakeup
        * @throws InterruptedException
        */
    public void waitMillis (double millis,
                            long startTimestamp,
                            WakeUp wakeup)
        throws InterruptedException
    {
        if (millis <= 0){
            return;
        }

        long timeStamp = startTimestamp;

        while (op.opModeIsActive() &&
            timeStamp - startTimestamp < millis) {

            if (wakeup != null && wakeup.isWakeUp()) {
                break;
            }

            //op.telemetry.addData("Wait", Long.toString((millis + startTimestamp - timeStamp) / 1000));
            op.idle();
            timeStamp = System.currentTimeMillis();
        }
    }

    /**
     * Callback for wakeup
     */
    public static interface WakeUp {
        public boolean isWakeUp();
    }
}
