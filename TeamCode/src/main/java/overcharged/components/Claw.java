package overcharged.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public OcServo claw;

    public static final float OPEN = 147f;
    public static final float GRAB = 117f;
    public static final float AUTO_GRAB = 123f;

    public Claw(HardwareMap hardwareMap) {
        claw = new OcServo(hardwareMap, "claw", OPEN);
    }

    public float getPos(){
        return claw.getPosition();
    }

    public void setOpen(){
        setPosition(OPEN);
    }

    public void setGrab(){
        setPosition(GRAB);
    }

    public void setAutoGrab(){
        setPosition(AUTO_GRAB);
    }

    public void setPosition(float pos){
        claw.setPosition(pos);
    }

}
