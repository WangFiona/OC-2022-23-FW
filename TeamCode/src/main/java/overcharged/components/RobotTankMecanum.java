package overcharged.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;
import overcharged.odometry.Localization;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_R;

/**
 * Overcharged Team #12599
 * Robot definition for Six wheel robot
 */
public class RobotTankMecanum {
    protected Telemetry telemetry;

    ///Drive components
    public OcMotor driveLeftFront = null;
    public OcMotor driveLeftBack = null;
    public OcMotor driveRightFront = null;
    public OcMotor driveRightBack = null;

    public Drive drive;

    ///Led indicator components
    private final OcLed ledYellow;
    private final OcLed ledGreen;
    private final OcLed ledWhite;
    private final OcLed ledBlue;
    private final OcLed ledRed;
    public final List<OcLed> leds = new ArrayList<>();
//    public QwiicLEDStrip ledStrip;

    ///Robot Gyro sensor
    public OcGyro2 gyroSensor;
    public Intake intake;
    public Arm arm;
    public Cup cup;
    public Duck duck;
    public oldSlide slide;
    public Cap cap;
    public OcSwitch limitSwitch;

    /// Odometry
    public Localization odometryLocalization = null;

    public DistanceSensor sensorDistance;
    public SensorDistance sensorDistanceR;
    public SensorDistance sensorDistanceL;
    public SensorDistance sensorDistanceF;

    public final List<OcServo> servos = new ArrayList<>();
    public final List<OcSwitch> switchs = new ArrayList<>();



    public RobotTankMecanum(OpMode op,
                            boolean isAutonomous)
    {
        this(op, isAutonomous, false);
    }



    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public RobotTankMecanum(OpMode op,
                        boolean isAutonomous, boolean lazy)
    {
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        RobotLog.ii(RobotConstants.TAG_R, "Initializing motors");
        ///Initialize Motors
        OcMotor driveLeftFront = null;
        if(!lazy) {
            try {
                driveLeftFront = new OcMotor(hardwareMap,
                        "driveLF",
                        DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveLF " + e.getMessage());
                missing = missing + "driveLF";
                numberMissing++;
            }
            this.driveLeftFront = driveLeftFront;

            OcMotor driveLeftBack = null;
            try {
                driveLeftBack = new OcMotor(hardwareMap,
                        "driveLB",
                        DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveLB " + e.getMessage());
                missing = missing + ", driveLB";
                numberMissing++;
            }
            this.driveLeftBack = driveLeftBack;

            OcMotor driveRightFront = null;
            try {
                driveRightFront = new OcMotor(hardwareMap,
                        "driveRF",
                        DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveRF " + e.getMessage());
                missing = missing + ", driveRF";
                numberMissing++;
            }
            this.driveRightFront = driveRightFront;

            OcMotor driveRightBack = null;
            try {
                driveRightBack = new OcMotor(hardwareMap,
                        "driveRB",
                        DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveRB " + e.getMessage());
                missing = missing + ", driveRB";
                numberMissing++;
            }
            this.driveRightBack = driveRightBack;

            if (isAutonomous) {
                OcGyro2 gyro2 = null;
                try {
                    gyro2 = new OcBnoGyro2(hardwareMap, "limu", "rimu");
                    while (isAutonomous && gyro2.isCalibrating()) {
                        telemetry.addData("Gyro", "Calibrating");
                        telemetry.update();
                        Thread.sleep(50);
                    }
                } catch (Exception e) {
                    RobotLog.ee(RobotConstants.TAG_R, "missing: gyro_sensor " + e.getMessage());
                    missing = missing + ", Gyro";
                    numberMissing++;
                }
                this.gyroSensor = gyro2;
            }
        }


        // intake initialization
        try {
            intake = new Intake(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: intake " + e.getMessage());
            missing = missing + ", Intake";
            numberMissing++;
        }

        // duck initialization
        try {
            duck = new Duck(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: duck " + e.getMessage());
            missing = missing + ", Duck";
            numberMissing++;
        }

        try {
            slide = new oldSlide(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: slide " + e.getMessage());
            missing = missing + ", Slide";
            numberMissing++;
        }

        try {
            cup = new Cup(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: cup " + e.getMessage());
            missing = missing + ", Cup";
            numberMissing++;
        }

        try {
            arm = new Arm(hardwareMap, isAutonomous);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: arm " + e.getMessage());
            missing = missing + ", arm";
            numberMissing++;
        }

        try {
            this.limitSwitch = new OcSwitch(hardwareMap,"limitswitch", true);
            boolean isSwitchNull= limitSwitch==null ? true : false;
            switchs.add(this.limitSwitch);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: limitSwitch " + e.getMessage());
            numberMissing++;
            boolean isSwitchNull= limitSwitch==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }

        try {
            sensorDistanceR = new SensorDistance(hardwareMap, "rangeR");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: distanceR" + e.getMessage());
            numberMissing++;
        }

        try {
            sensorDistanceL = new SensorDistance(hardwareMap, "rangeL");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: distanceL" + e.getMessage());
            numberMissing++;
        }

        try {
            sensorDistanceF = new SensorDistance(hardwareMap, "rangeF");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: distanceF" + e.getMessage());
            numberMissing++;
        }

        try {
            cap = new Cap(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: cap " + e.getMessage());
            missing = missing + ", cap";
            numberMissing++;
        }

        if(!lazy){
            this.drive = createDrive();
        }

        ///Initialize Leds
        RobotLog.ii(TAG_R,  "Initializing Leds");
        OcLed led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_yellow");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_yellow " + e.getMessage());
            missing = missing + ", led_yellow";
            numberMissing++;
        }
        ledYellow = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_green");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_green " + e.getMessage());
            missing = missing + ", led_green";
            numberMissing++;
        }
        ledGreen = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_white");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_white " + e.getMessage());
            missing = missing + ", led_white";
            numberMissing++;
        }
        ledWhite = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_blue");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_blue " + e.getMessage());
            missing = missing + ", led_blue";
            numberMissing++;
        }
        ledBlue = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_red");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_red " + e.getMessage());
            missing = missing + ", led_red";
            numberMissing++;
        }
        ledRed = led;

//        QwiicLEDStrip led = null;
//        try {
//            led = hardwareMap.get(QwiicLEDStrip.class, "ledstrip");
//            led.setBrightness(8);
//        } catch (Exception e) {
//            RobotLog.ee(TAG_R,  "missing: ledstrip " + e.getMessage());
//            missing = missing + ", led";
//            numberMissing++;
//        }
//        this.ledStrip = led;

        RobotLog.ii(RobotConstants.TAG_R,  "Initializing RobotMecanum complete");
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.addData("Missing", missing);
        telemetry.update();
    }

    /**
     * Robot and sensor shut down
     */
    public void close ()
    {
        if (this.drive != null) {
            this.drive.stop();
        } else {
            if (this.driveLeftFront != null) this.driveLeftFront.setPower(0f);
            if (this.driveLeftBack != null) this.driveLeftBack.setPower(0f);
            if (this.driveRightFront != null) this.driveRightFront.setPower(0f);
            if (this.driveRightBack != null) this.driveRightBack.setPower(0f);
        }

    }

    /**
     * subclass override this method
     * @return Drive
     */
    protected Drive createDrive () {
        return new TankDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);
    }

    /**
     * initialize tank drive
     * @return TankDrive
     */
    public TankDrive getTankDrive () {
        return (TankDrive) this.drive;
    }

    /**
     * Proxy methods
     */
    public void intakeOn(){
        intake.on();
    }
    public void intakeOff(){
        intake.off();
    }
    public void outtake(){
        intake.out();
    }

    public void slideUp(){ slide.up();}
    public void slideDown(){ slide.down();}
    public void slideOn(double power){ slide.on(power);}
    public void slideMid(){ slide.mid();}
    public void slideOff(){ slide.off();}
    public void slideBrake(){slide.stop();}
    public void slidePosition(int pow){slide.position(pow);}

    public void duckOn(){ duck.on();}
    public void duckOn(double power){ duck.on(power);}
    public void duckMid(){ duck.mid();}
    public void duckMax(){duck.max();}
    public void duckOff(){ duck.off();}

    public void cupOpen(){cup.setOpen();}
    public void cupDump(){cup.setDump();}
    public void cupLocked(){cup.setLocked();}

    public void armDown(){arm.setDown();}
    public void armMid(){arm.setMid();}
    public void armOut(){arm.setOut();}
    public void armOutShared(){arm.setOutShared();}
    public boolean isArmOut(){ return arm.armOut; }
    public boolean isArmDown(){ return arm.armDown; }
    public void armOutAuto(){arm.setAutoOutforA();}

    public boolean isCollected(){ return cup.isCollected(); }
    public Cup.FreightType getFreight(){ return cup.getFreight(); }

    public boolean isSlideSwitchPressed(){ return limitSwitch.isTouch(); }

    public double getSlidePosition(){ return slide.getSlidePosition(); }
    public void resetSlidePosition(){ slide.resetSlidePosition(); }
    public double getDistanceR(){ return sensorDistanceR.getDistance(); }
    public double getDistanceL(){ return sensorDistanceL.getDistance(); }
    public double getDistanceF(){ return sensorDistanceF.getDistance(); }

    public void setCapPos(float pos){ cap.setCapPos(pos); }
    public void setCapOut(){ cap.setCapOut(); }
    public void setCapIn(){ cap.setCapIn(); }
    public void setCapUp(){ cap.setCapUp(); }
    public void setCapped(){ cap.setCapped(); }

    /**
     * update LEDs
     */
    public void drawLed () {
        for (OcLed led: leds) {
            led.draw();
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if true turn on
     */
    public void ledYellowOn(boolean on) {
        try {
            if (on) {
                this.ledYellow.on();
            } else {
                this.ledYellow.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_yellow " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if true turn on
     */
    public void ledBlueOn(boolean on) {
        try {
            if (on) {
                this.ledBlue.on();
            } else {
                this.ledBlue.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Blue " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if true turn on
     */
    public void ledGreenOn(boolean on) {
        try {
            if (on) {
                this.ledGreen.on();
            } else {
                this.ledGreen.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Green " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledGreenBlink() {
        try {
            this.ledGreen.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Green " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledBlueBlink() {
        try {
            this.ledBlue.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Blue " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledRedBlink() {
        try {
            this.ledRed.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Red " + e.getMessage());
        }
    }
    /**
     * Protect the led access and blink
     */
    public void ledWhiteBlink() {
        try {
            this.ledWhite.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_White " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledYellowBlink() {
        try {
            this.ledYellow.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Yellow " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void ledRedOn(boolean on) {
        try {
            if (on) {
                this.ledRed.on();
            } else {
                this.ledRed.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Red " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else alse
     */
    public void ledWhiteOn(boolean on) {
        try {
            if (on) {
                this.ledWhite.on();
            } else {
                this.ledWhite.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_White " + e.getMessage());
        }
    }
}