package overcharged.components;

/**
 * General Button definitions
 */
public class Button
{
    public final static Button BTN_BOTTOM = new Button();
    public final static Button BTN_L1 = new Button();
    public final static Button BTN_L2 = new Button();
    public final static Button BTN_L3 = new Button();
    public final static Button BTN_L4 = new Button();
    public final static Button BTN_SLIDEON = new Button();
    public final static Button BTN_SLIDEDOWN = new Button();
    public final static Button BTN_TURRET = new Button();
    public final static Button BTN_TEST = new Button();
    public final static Button BTN_TEST1 = new Button();
    public final static Button TURRET_RESET = new Button();
    public final static Button SLIDE_RESET = new Button();
    public final static Button LOWER = new Button();
    public final static Button TURRET_LEFT = new Button();
    public final static Button TURRET_RIGHT = new Button();
    public final static Button SET_ZERO = new Button();
    public final static Button RESET_H = new Button();


    public final static Button BTN_STRAIGHTEN = new Button();

    public final static Button BTN_PLUS = new Button();
    public final static Button BTN_MINUS = new Button();
    public final static Button BTN_NEXT = new Button();
    public final static Button BTN_POP = new Button();
    public final static Button BTN_PREV = new Button();
    public final static Button BTN_START = new Button();
    public final static Button BTN_BACK = new Button();
    public final static Button BTN_MIN = new Button();
    public final static Button BTN_MAX = new Button();
    public final static Button BTN_TUCK_IN = new Button();
    public final static Button BTN_LATCH = new Button();
    public final static Button BTN_LATCH_READY = new Button();
    public final static Button BTN_MID = new Button();
    public final static Button BTN_IN = new Button();
    public final static Button BTN_OUT = new Button();
    public final static Button BTN_INTAKE = new Button();
    public final static Button BTN_OUTTAKE = new Button();
    public final static Button BTN_OPEN = new Button();
    public final static Button BTN_HORIZONTAL = new Button();
    public final static Button BTN_CLOSE = new Button();
    public static final Button BTN_VERTICAL = new Button(); //B
    public static final Button BTN_TANK = new Button();  //A
    public static final Button BTN_SNAKE = new Button();  //X
    public static final Button BTN_CRAB = new Button();  //X
    public static final Button BTN_SWERVE = new Button();  //B
    public static final Button BTN_AUTOMOBILE = new Button(); //B
    public static final Button BTN_HANG = new Button(); //y
    public static final Button BTN_INITAKE_LIMIT = new Button(); //y
    public static final Button BTN_SUPERSLOW_MODE = new Button(); //LB
    public static final Button BTN_SLOW_MODE = new Button(); //RB
    public static final Button BTN_MARKER_DUMPER = new Button(); //DPAD UP/DOWN
    public static final Button BTN_DUMPER = new Button();
    public static final Button BTN_AUTO = new Button();
    public static final Button BTN_STICK = new Button();
    public static final Button BTN_INTAKE_HOLDER = new Button();
    public static final Button BTN_LENGTHWISE = new Button();
    public static final Button BTN_CAPSTONE = new Button();

    /// Virtual Fourbar
    public static final Button BTN_DOWN = new Button();
    public static final Button BTN_UP = new Button();
    public static final Button BTN_LEVEL1 = new Button();
    public static final Button BTN_LEVEL2 = new Button();
    public static final Button BTN_LEVEL3 = new Button();
    //public static final Button BTN_BOTTOM = new Button();
    public static final Button BTN_FOURBAR = new Button();
    public static final Button BTN_FOURBAR_ROTATE = new Button();
    public static final Button BTN_AUTOMATION = new Button();

    public static final Button BTN_SLIDE = new Button();
    public static final Button BTN_SLIDE_UP = new Button();
    public static final Button BTN_SLIDE_DOWN = new Button();

    ///Assign buttons to intake controls
    public static final Button BTN_COLLECT = new Button(); //RT2
    public static final Button BTN_REJECT = new Button(); //LT2
    public static final Button BTN_SPEED = new Button(); //x, b

    public static final Button BTN_POWERSHOT = new Button();
    public static final Button BTN_FLYWHEEL = new Button();
    public static final Button BTN_WOBBLE_CLAW = new Button();
    public static final Button BTN_WOBBLE_ARM = new Button();
    public static final Button BTN_WOBBLE_UP = new Button();


    public static final int BTN_PRESS_INTERVAL = 500; // milliseconds
    public static final float TRIGGER_THRESHOLD = (float)0.75;
    public static final Button BTN_DISABLE = new Button();
    public static final Button BTN_RIGHT = new Button();
    public static final Button BTN_LEFT = new Button();

    public long lastPressTime;

    /**
     * standard press function.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPress(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL);
    }

    /**
     * press function for half time.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPressShort(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/2);
    }

    /**
     * press function for fourth time.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPress4Short(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/4);
    }

    /**
     * press function for eighth time.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPress6Short(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/6);
    }

    /**
     * determine if the button has become pressable.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @param interval interval from previous button press
     * @return if the button is pressable yet
     */
    public boolean canPress(long timeStamp,
                            long interval) {
        boolean pressable = isPreesable(timeStamp, interval);
        if(pressable) {
            lastPressTime = timeStamp;
        }
        return pressable;
    }

    /**
     * determine if the button has become pressable
     * without updating internal timestamp
     * @param timeStamp
     * @param interval
     * @return
     */
    public boolean isPreesable(long timeStamp,
                               long interval) {
        return (timeStamp - lastPressTime) > interval;
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     *
     * fVal            fScale          scaleArray
     * -1.0            -1.0            -1.0
     * -0.975          -1.0            -1.0
     * -0.95000005     -1.0            -1.0
     * -0.9250001      -0.9540002      -0.85
     * -0.9000001      -0.9020002      -0.85
     * -0.8750001      -0.85000026     -0.85
     * -0.85000014     -0.7920003      -0.72
     * -0.82500017     -0.7440004      -0.72
     * -0.8000002      -0.68000036     -0.6
     * -0.7750002      -0.6400004      -0.6
     * -0.75000024     -0.6000004      -0.6
     * -0.72500026     -0.5420003      -0.5
     * -0.7000003      -0.5140003      -0.5
     * -0.6750003      -0.48600036     -0.43
     * -0.65000033     -0.4580004      -0.43
     * -0.62500036     -0.4300004      -0.43
     * -0.6000004      -0.3960004      -0.36
     * -0.5750004      -0.3720004      -0.36
     * -0.5500004      -0.34800044     -0.3
     * -0.52500045     -0.32400045     -0.3
     * -0.5000005      -0.30000046     -0.3
     * -0.47500047     -0.27600044     -0.24
     * -0.45000046     -0.25200045     -0.24
     * -0.42500046     -0.20400023     -0.18
     * -0.40000045     -0.19200023     -0.18
     * -0.37500045     -0.18000022     -0.18
     * -0.35000044     -0.16800022     -0.15
     * -0.32500044     -0.15600021     -0.15
     * -0.30000043     -0.13600013     -0.12
     * -0.27500042     -0.12800013     -0.12
     * -0.25000042     -0.12000013     -0.12
     * -0.22500041     -0.106000066    -0.1
     * -0.2000004      -0.102000065    -0.1
     * -0.1750004      -0.12200026     -0.09
     * -0.1500004      -0.10600026     -0.09
     * -0.12500039     -0.09000025     -0.09
     * -0.10000039     -0.08000031     -0.05
     * -0.07500039     -0.060000315    -0.05
     * -0.050000392    0.0             -0.0
     * -0.025000392    0.0             -0.0
     * -3.91528E-7     0.0             -0.0
     * 0.02499961      0.0             0.0
     * 0.04999961      0.0             0.0
     * 0.07499961      0.05999969      0.05
     * 0.09999961      0.079999685     0.05
     * 0.124999605     0.09999968      0.05
     * 0.1499996       0.10599975      0.09
     * 0.17499961      0.121999756     0.09
     * 0.19999962      0.10199994      0.1
     * 0.22499962      0.10599994      0.1
     * 0.24999963      0.10999994      0.1
     * 0.27499962      0.12799987      0.12
     * 0.29999962      0.13599987      0.12
     * 0.32499963      0.15599982      0.15
     * 0.34999964      0.16799983      0.15
     * 0.37499964      0.17999984      0.15
     * 0.39999965      0.19199984      0.18
     * 0.42499965      0.20399985      0.18
     * 0.44999966      0.25199968      0.24
     * 0.47499967      0.27599967      0.24
     * 0.49999967      0.29999965      0.24
     * 0.5249997       0.3239997       0.3
     * 0.54999965      0.3479997       0.3
     * 0.57499963      0.37199965      0.36
     * 0.5999996       0.39599964      0.36
     * 0.6249996       0.4199996       0.36
     * 0.64999956      0.4579995       0.43
     * 0.67499954      0.48599946      0.43
     * 0.6999995       0.51399946      0.5
     * 0.7249995       0.5419994       0.5
     * 0.74999946      0.5699994       0.5
     * 0.77499944      0.63999915      0.6
     * 0.7999994       0.6799991       0.6
     * 0.8249994       0.7439989       0.72
     * 0.84999937      0.7919988       0.72
     * 0.87499934      0.8399988       0.72
     * 0.8999993       0.90199864      0.85
     * 0.9249993       0.95399857      0.85
     * 0.9499993       1.0             1.0
     * 0.97499925      1.0             1.0
     * 0.9999992       1.0             1.0
     */
    public static float scaleInput(float fVal)  {
        final float[] scaleArray = {0.0f, 0.05f, 0.09f, 0.10f, 0.12f, 0.15f, 0.18f, 0.24f,
                0.30f, 0.36f, 0.43f, 0.50f, 0.60f, 0.72f, 0.85f, 1.00f, 1.00f};

        // get the corresponding index for the scaleInput array.
        float index = Math.abs(fVal) * 16f;
        int index1 = (int)index;
        if (index1 >= 16) {
            return Math.signum(fVal);
        } else if (index1 == 0) {
            return 0;
        }
        int index2 = index1 - 1;

        // Y = (X - x1) * (y2 - y1) / (x2 - x1) + y1
        float fScale = (index - index1) * (scaleArray[index2] - scaleArray[index1]) / (index2 - index1) + scaleArray[index1];
        if (fScale > 1f) {
            fScale = 1f;
        }

        return Math.signum(fVal) * fScale;
    }
}
