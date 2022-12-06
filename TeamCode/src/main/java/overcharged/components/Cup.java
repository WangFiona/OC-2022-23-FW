package overcharged.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Cup {
    public Servo cup;
    public RevColorSensorV3 colorSensor;

    public static final double open = 0.25;
    public static final double lockedBlock = 0.12;
    public static final double lockedBall = 0.17;
    public static final double lockedDuck = 0.06;
    public static final double dump = 0.42;

    public static final double ballLight = 2020;
    ///empty cup 7.52cm - 8.95cm, block 1.85cm - 3.24cm, ball 0.64cm - 1.17cm
    ///based on the above readings its safe to assume that if the distance is less than 7.52cm,
    ///something is collected. But to be safe setting the value to 5
    public static final double distance = 5;

    public Cup(HardwareMap hardwareMap){
        cup = hardwareMap.servo.get("cup");
        cup.setPosition(open);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public double getPos(){
        return cup.getPosition();
    }

    public void setDump(){
        cup.setPosition(dump);
    }

    public void setLocked(){
        if(colorSensor.getRawLightDetected() >= ballLight){
            //if the distance is greater than a block then its a duck
            cup.setPosition(lockedBall);
        } else {
            cup.setPosition(lockedBlock);
        }
    }

    public void setDuckLocked() { cup.setPosition(lockedDuck); }

    public enum FreightType
    {
        BLOCK,
        BALL,
        DUCK
    }

    public FreightType getFreight(){
        if(colorSensor.getRawLightDetected() >= ballLight){
            return FreightType.BALL;
        } else {
            return FreightType.BLOCK;
        }
    }

    public void setOpen(){
        cup.setPosition(open);
    }

    public void setPosition(double pos){
        cup.setPosition(pos);
    }

    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    public boolean isCollected(){
        return getDistance() < distance;
    }

    public boolean isDuckCollected(){
        return getDistance() < 6.0;
    }
}
