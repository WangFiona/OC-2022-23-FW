package overcharged.linear.components;

/**
 * Action to be performed while robot is moving.
 */
public interface MoveAction {

    /**
     * get distance in encoder ticks.
     * @return distance in encoder ticks.
     */
    int distanceInTick();

    /**
     * perform action.  Note action should be idempotent
     */
    void perform();

    /**
     * stop action
     */
    void stop();
}
