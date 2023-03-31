package overcharged.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public OcServo claw;

    public static final float OPEN = 210f;//208f;//142f;//134f;
    public static final float GRAB = 171f;//165f;//108f;//86f;
    public static final float AUTO_OPEN = 223f;//247f;//182f;//170f;

    public Claw(HardwareMap hardwareMap) {
        claw = new OcServo(hardwareMap, "claw", 217f);
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

    public void setAutoOpen(){
        setPosition(AUTO_OPEN);
    }

    public void setPosition(float pos){
        claw.setPosition(pos);
    }

}
