package overcharged.linear.util;

import overcharged.components.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * For selection using game pad before waitForStart()
 */
public class SelectLinear
{
    private LinearOpMode op;

    public SelectLinear(LinearOpMode op) {
        this.op = op;
    }

    /**
     * Adjust delay in milliseconds
     * @return delay in milliseconds
     * @throws InterruptedException
     */
    public double adjustDelay ()
        throws InterruptedException {
        RobotLog.i("entered adjust");
        // 30 second max
        return adjust("Delay", 30) * 1000;
    }

    /**
     * Adjust a number
     * @return a number
     * @throws InterruptedException
     */
    public double adjust (String title,
                       int max)
        throws InterruptedException
    {
        op.gamepad1.reset();
        double number = 0;
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if (op.gamepad1.b && Button.BTN_PLUS.canPressShort(timeStamp)) {
                number += 0.5;
                if (number > max) {
                    number = max;
                }
            } else if (op.gamepad1.x && Button.BTN_MINUS.canPressShort(timeStamp)) {
                number -= 0.5;
                if (number < 0) {
                    number = 0;
                }
            }
            else if (op.gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }

            op.telemetry.addData(title + " adjust", "+:B  -:X");
            op.telemetry.addData("Confirm", "Start");
            op.telemetry.addData(title, number);
            op.telemetry.update();

            op.idle();
        }

        RobotLog.i("Adjust: " + title + "=" + number);
        op.idle();
        return number;
    }

    /**
     * Adjust a number
     * @param title display title
     * @param min minimum value to choose
     * @param max maximim value to choose
     * @param def default value to set
     *  @return a number between min and max
     * @throws InterruptedException
     */
    public int adjust (String title,
                       int min, int max, int def)
            throws InterruptedException
    {
        op.gamepad1.reset();
        int number = def;
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if (op.gamepad1.b && Button.BTN_PLUS.canPressShort(timeStamp)) {
                number += 1;
                if (number > max) {
                    number = max;
                }
            } else if (op.gamepad1.x && Button.BTN_MINUS.canPressShort(timeStamp)) {
                number -= 1;
                if (number < min) {
                    number = min;
                }
            }
            else if (op.gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }

            op.telemetry.addData(title + " adjust", "+:B  -:X");
            op.telemetry.addData("Confirm", "Start");
            op.telemetry.addData(title, number);
            op.telemetry.update();

            op.idle();
        }

        RobotLog.i("Adjust: " + title + "=" + number);
        op.idle();
        return number;
    }
    /**
     * Select alliance
     * @return true for red, false for blue
     * @throws InterruptedException
     */
    public boolean selectAlliance ()
        throws InterruptedException {
        RobotLog.i("entered alliance selection");

        String[] alliances = new String[] {"Blue", "Red"};
        int index = select(alliances,
                           "Alliance");
        return index == 0;
    }

    public boolean selectExtraDistance ()
            throws InterruptedException {
        RobotLog.i("entered alliance selection");

        String[] alliances = new String[] {"Extra", "No Extra"};
        int index = select(alliances,
                "Extra Distance");
        return index == 0;
    }

    public int selectLength ()
            throws InterruptedException {
        RobotLog.i("entered hslide dump length");

        String[] length = new String[] {"Medium", "Far", "Close"};
        //int index =
        int index = select(length,
                "Dump Length");
        return index;
    }

    public boolean selectWarehouse ()
            throws InterruptedException {

        String[] warehouse = new String[] {"Warehouse", "Storage"};
        int index = select(warehouse,
                "Warehouse or Storage");
        return index == 0;
    }

    public boolean selectDetectorPosition ()
            throws InterruptedException {

        String[] positions = new String[] {"Right", "Left"};
        int index = select(positions,
                "Ring Detector Position");
        return index == 0;
    }

    public boolean selectPowerShots ()
            throws InterruptedException {

        String[] powerShot = new String[] {"Yes", "No"};
        int index = select(powerShot,
                "Power Shots");
        return index == 0;
    }

    public boolean selectHighGoal ()
            throws InterruptedException {

        String[] highGoal = new String[] {"Yes", "No"};
        int index = select(highGoal,
                "High Goal");
        return index == 0;
    }

    public boolean selectWobbleGoal ()
            throws InterruptedException {

        String[] wobbleGoal = new String[] {"Yes", "No"};
        int index = select(wobbleGoal,
                "Wobble Goal");
        return index == 0;
    }

    public boolean selectWobbleGoalNumber ()
            throws InterruptedException {

        String[] wobbleGoal = new String[] {"1", "2"};
        int index = select(wobbleGoal,
                "Number of Wobble Goals");
        return index == 0;
    }

    /**
     * Select position
     * @return true for near, false for far
     * @throws InterruptedException
     */
    public boolean selectPosition ()
            throws InterruptedException {

        String[] positions = new String[] {"Left", "Right"};
        int index = select(positions,
                "Position");
        return index == 0;
    }

    /**
     * Select parking position when robot starts at depot side
     * @return true for own side, false for opposite side
     * @throws InterruptedException
     */
    public boolean selectParkingPosition ()
            throws InterruptedException {

        String[] positions = new String[] {"Own Side", "Opposite Side"};
        int index = select(positions,
                "Parking At");
        return index == 0;
    }

    /**
     * Select whether or not to knock of partner's mineral when robot starts at crater side
     * @return true for own side, false for opposite side
     * @throws InterruptedException
     */
    public boolean selectKnockOff2ndMineral ()
            throws InterruptedException {

        String[] positions = new String[] {"No", "Yes"};
        int index = select(positions,
                "Knock Off Partner's Mineral");
        return index == 1;
    }
	
	 public boolean selectOnlyPark ()
            throws InterruptedException {

        String[] positions = new String[] {"No", "Yes"};
        int index = select(positions,
                "Only do the parking");
        return index == 1;
    }

    public boolean selectCollectStone ()
        throws InterruptedException {
        String[] positions = new String[] {"No", "Yes"};
        int index = select(positions,
                "Collect skystone");
        return index == 1;
    }

    /**
     * select the collect stone
     * @param def default index
     * @return
     * @throws InterruptedException
     */
    public boolean selectCollectStone (int def)
            throws InterruptedException {
        return selectYNStuff("Collect skystone", def);
    }

    /**
     * Select Yes/No for the specified  stuff
     * @param title title to display
     * @param def default index
     * @return
     * @throws InterruptedException
     */
    public boolean selectYNStuff (String title, int def)
            throws InterruptedException {
        return selectStuff(new String[] {"No", "Yes"},title, def);
    }

    /**
     * select the collect stone
     * @param def default index
     * @return
     * @throws InterruptedException
     */
    public boolean selectStuff (String[] positions, String title, int def)
            throws InterruptedException {
        int index = select(positions, title, def);
        return index == 1;
    }

    /**
     * Select hanging position
     * @return true for Not Hanging, false for Hanging
     * @throws InterruptedException
     */
    public boolean selectHangingPosition ()
            throws InterruptedException {

        String[] positions = new String[] {"Hanging", "Ground"};
        int index = select(positions,
                "Position");
        return index == 1;
    }

    /**
     * Select backoff distance
     * @return true for near, false for far
     * @throws InterruptedException
     */
    public boolean selectPark ()
            throws InterruptedException {

        String[] parkNear = new String[] {"Far", "Near"};
        int index = select(parkNear,
                "Navigate under skybridge. Park");
        return !(index == 0);
    }

    /**
     * Select move foundation or not
     * @return true for Yes, false for NO
     * @throws InterruptedException
     */
    public boolean selectMoveFoundation (int def)
            throws InterruptedException {
        return selectYNStuff("Move the Foundation", def);
    }

    /**
     * Select whether to collect stones or not
     * @return true for Yes, false for NO
     * @throws InterruptedException
     */
    public boolean selectcollectStones ()
            throws InterruptedException {

        String[] collectStones = new String[] {"Yes", "No"};
        int index = select(collectStones,
                "Collect Quarry Stones");
        return index == 0;
    }

    /**
     * Select backoff distance
     * @return true for near, false for far
     * @throws InterruptedException
     */
    public boolean selectBackoff ()
            throws InterruptedException {

        String[] backoffNear = new String[] {"Near", "Far"};
        int index = select(backoffNear,
                "Backoff Distance");
        return index == 0;
    }

    /**
     * Select from the given array
     * @return index from the given array starting 0
     * @throws InterruptedException
     */
    public int select (Object[] list,
                       String title)
            throws InterruptedException {
        return select(list, title, 0);
    }
    /**
     * Select from the given array
     * @return index from the given array starting 0
     * @throws InterruptedException
     */
    public int select (Object[] list,
                       String title, int index)
        throws InterruptedException {

        op.gamepad1.reset();
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();
            if(op.gamepad1.b && Button.BTN_NEXT.canPress(timeStamp)) {
                index++;
                if(index >= list.length){
                    index = 0;
                }
            } else if(op.gamepad1.x && Button.BTN_PREV.canPress(timeStamp)) {
                index--;
                if(index < 0){
                    index = list.length - 1;
                }
            }
            else if (op.gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }

            op.telemetry.addData(title, list[index]);
            op.telemetry.addData("Select", "Next: B  Prev: X");
            op.telemetry.addData("Confirm", "Start");
            op.telemetry.update();

            op.idle();
        }

        RobotLog.i("Select: " + title + "=" + list[index]);
        op.idle();
        return index;
    }

    /**
     * Confirm selection
     * @param title item to be confirmed
     * @param def default value
     * @return true to confirm
     */
    public boolean confirm (String title,
                            boolean def)
        throws InterruptedException {

        String[] options;
        if (def) {
            options = new String[]{"Yes", "No"};
        } else {
            options = new String[]{"No", "Yes"};
        }

        int index = select(options,
                           title);
        return index == (def ? 0 : 1);
    }
}
