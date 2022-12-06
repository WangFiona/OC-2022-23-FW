package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CupSixWheel {
    Servo cup;

    public static double dump = 0.63f;
    public static double locked = 0.83;
    public static double open = 0.9;

    public CupSixWheel(HardwareMap hardwareMap){
        cup = hardwareMap.servo.get("cup");

        cup.setDirection(Servo.Direction.FORWARD);
        cup.setPosition(open);
    }

    public void setDump(){
        cup.setPosition(dump);
    }

    public void setLocked(){
        cup.setPosition(locked);
    }

    public void setOpen(){
        cup.setPosition(open);
    }

    public void setPosition(double pos){
        cup.setPosition(pos);
    }

    public void disable() {
        cup.getController().pwmDisable();
    }

    public void enable() {
        cup.getController().pwmEnable();
    }
}