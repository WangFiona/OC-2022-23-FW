package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Cap {
    public Servo cap;
    public static double in = 0.79; //0.8
    public static double out = 0.60; //0.61
    public static double up = 0.24; //0.25
    public static double capped = 0.40; //0.41
    public boolean isCapOut = false;

    public Cap(HardwareMap hardwareMap){
        cap = hardwareMap.servo.get("cap");
        cap.setPosition(in);
        //cap.scaleRange(0.4, 1);
    }

    public void setCapPos(float pos){
        if(isCapOut){
            double position = cap.getPosition()+(pos*0.008);
            position = position <= up ? up : position;
            cap.setPosition(position);
        }
    }

    public void setCapped(){
        if(isCapOut) cap.setPosition(capped);
    }

    public void setCapOut(){
        cap.setPosition(out);
        isCapOut = isOut();
    }

    public void setCapIn(){
        cap.setPosition(in);
        isCapOut = isOut();
    }

    public void setCapUp(){
        cap.setPosition(up);
        isCapOut = isOut();
    }

    public boolean isOut(){
        if(cap.getPosition() <= out) return true;
        else return false;
    }

}
