package overcharged.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import static overcharged.config.RobotConstants.TAG_H;

/**
 * Created by Kevin on 11/22/2017.
 */
public class OcSwitch
    extends OcDevice
{
    private AnalogInput analog;
    private DigitalChannel digital;
    private boolean isDigital;
	private boolean disabled = false;

    /**
     * initialize the switches
     * @param hardwareMap HardwareMap to get motor from
     * @param switchName name of switches
     * @param isDigital digital port or not
     */
    public OcSwitch(HardwareMap hardwareMap,
                    String switchName,
                    boolean isDigital) {
        super(switchName);
        RobotLog.ii(TAG_H, "Constructor for Switch " + switchName + " isDigital=" + isDigital);
        this.isDigital = isDigital;
        if (isDigital) {
            /// get a reference to our digitalTouch object.
            digital = hardwareMap.get(DigitalChannel.class, switchName);
            boolean isNull= digital==null ? true : false;
            RobotLog.ii(TAG_H, "digital? " + isNull);

            /// set the digital channel to input.
            digital.setMode(DigitalChannel.Mode.INPUT);
        }
        else {
            analog = hardwareMap.get(AnalogInput.class, switchName);
        }
    }

    /**
     * Disable this switch if the caller thinks its not functional during operation
     * @return status of the switch
     */
    public void disable () {
		disabled = true;
    }

    /**
     * determines if the switch has been disabled, normally by the caller
     * @return status of the switch
     */
    public boolean isDisabled () {
        return disabled;
    }

    /**
     * determines if the switch is pressed
     * @return status of the switch
     */
    public boolean isTouch () {
        if (isDigital) {
			return !(!disabled ? digital.getState() : false); //switched for new limit switch (when clicked, false is returned)
       }
        else {
            return analog.getVoltage() / analog.getMaxVoltage() > 0.99;
        }
    }
}