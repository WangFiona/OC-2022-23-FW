package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class hSlides {
    public OcServo hSlides;

    public static final float IN = 33f;//44f;//185f;
    public static final float INIT = IN;
    public static final float START = 149f;//160f;
    public static final float PRESET1 = 142f;//153f;
    public static final float OUT = 179f;//190f;//60f;//70f;

    public hSlides(HardwareMap hardwareMap){
        hSlides = new OcServo(hardwareMap, "hSlides", INIT);
    }
    public float getPos(){
        return hSlides.getPosition();
    }

    public void setInit(){
        setPosition(INIT);
    }

    public void setPosition(float pos){
        hSlides.setPosition(pos);
    }
}
