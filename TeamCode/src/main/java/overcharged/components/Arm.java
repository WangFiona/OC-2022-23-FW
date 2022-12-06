package overcharged.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    public Servo left;
    public Servo right;

    public double downR = 0.07;
    public double downL = 1.0;

    public static final double mid = 0.21;
    public static final double out = 0.36;
    public static final double outShared = 0.4;
    public static final double outSharedReach = 0.5;
    public static final double outSharedReachLess = 0.45;

    public static final double outSharedAutoR = 0.643;
    public static final double outSharedAutoL = 0.427;



    public static final double autoOut = 0.373;
    public static final double autoOutforR = 0.621;
    public static final double autoOutforL = 0.449;

    public static final double autoOutforA = 0.42;
    public static final double autoOutforAR = 0.668;
    public static final double autoOutforAL = 0.402;

    public boolean armOut;
    public boolean armDown;

    //double adjust = 0.0615;

    public static final double adjust = 0.20132218;//0.5454;

    public Arm(HardwareMap hardwareMap, boolean isAutonomous){
        downR += adjust;
        downL -= adjust;

        right = hardwareMap.servo.get("armR");
        left = hardwareMap.servo.get("armL");
        if(isAutonomous){
            setMid();
            armDown = false;
        } else {
            setDown();
            armDown = true;
        }

        armOut = false;
    }

    public void setDown(){
        setPosition(0);
        armOut = false;
        armDown = true;
    }

    public void setMid(){
        setPosition(mid);
        armOut = false;
        armDown = false;
    }
    public double getMid(){
        return mid;
    }
    public void setOut(){
        setPosition(out);
        armOut = true;
        armDown = false;
    }

    public void setOutShared(){
        setPosition(outShared);
        armOut = true;
        armDown = false;
    }

    public void setOutSharedReach(){
        setPosition(outSharedReach);
        armOut = true;
        armDown = false;
    }

    public void setOutSharedReachLess(){
        setPosition(outSharedReachLess);
        armOut = true;
        armDown = false;
    }

    public void setAutoOut() {
        setOut();/*
        left.setPosition(autoOutforL);
        right.setPosition(autoOutforR);
        armOut = true;
        armDown = false;*/
    }

    public void setAutoShared(){
        setOutShared(); /*
        left.setPosition(outSharedAutoL);
        right.setPosition(outSharedAutoR);
        armOut = true;
        armDown = false;*/
    }

    public void setAutoOutforA() {
        setOutShared(); /*
        left.setPosition(autoOutforAL);
        right.setPosition(autoOutforAR);
        armOut = true;
        armDown = false;*/
    }

    public void setPosition(double pos){
        left.setPosition(downL-pos);
        right.setPosition(downR+pos);
    }

    public void setAdjust(double adjust){
        downL -= adjust;
        downR += adjust;

        setDown();
    }

    public void move(double amount){
        amount *= 0.007;

        left.setPosition(left.getPosition() - amount);
        right.setPosition(right.getPosition() + amount);


    }




}
