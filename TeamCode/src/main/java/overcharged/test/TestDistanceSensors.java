package overcharged.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import overcharged.components.Button;
import overcharged.components.OcSwitch;

/**
 * Created by Parthiv Nair on 3/8/2022.
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestDistanceSensors", group = "Test")
public class TestDistanceSensors extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private OcSwitch ocSwitch;
    RevColorSensorV3 colorSensor;
    double ballLight = 2020;
    double distance = 2.5;
    Rev2mDistanceSensor rangeR;
    Rev2mDistanceSensor rangeL;
    Rev2mDistanceSensor rangeF;

    AnalogInput sonar;

    double cup_min = 99;
    double cup_current = 0;
    double cup_max = -99;


    @Override
    public void init() {

        hardwareMap.logDevices();
        //Initialize Switch
        try {
            sonar = hardwareMap.get(AnalogInput.class, "sonar");
            colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
            rangeR = hardwareMap.get(Rev2mDistanceSensor.class, "rangeR");
            rangeL = hardwareMap.get(Rev2mDistanceSensor.class, "rangeL");
            rangeF = hardwareMap.get(Rev2mDistanceSensor.class, "rangeF");
            ocSwitch = new OcSwitch(hardwareMap,
                    "limitswitch",
                    true);
            telemetry.addData("Status", "Initialized " + runtime.toString());
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    public void start() {
        runtime.reset();    // Start game timer.
    }

    @Override
    public void loop() {
        long timeStamp = System.currentTimeMillis();
        cup_current = colorSensor.getDistance(DistanceUnit.CM);
        if (cup_current < cup_min) {
            cup_min = cup_current;
        }
        if (cup_current > cup_max) {
            cup_max = cup_current;
        }
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Cup", "%.2f cm", cup_current);
        telemetry.addData("Cup Min", "%.2f cm", cup_min);
        telemetry.addData("Cup Max", "%.2f cm", cup_max);
        telemetry.addData("rangeR", "%.2f in", rangeR.getDistance(DistanceUnit.INCH));
        telemetry.addData("rangeF", "%.2f in", rangeF.getDistance(DistanceUnit.INCH));
        telemetry.addData("rangeL", "%.2f in", rangeL.getDistance(DistanceUnit.INCH));

        float sonarDistance = (float) (((sonar.getVoltage()*6f*1024f/2.7f)-300f)/25.4f);
        float sonarActualDistance = (float) (0.9829f * sonarDistance -0.5991);
        telemetry.addData("sonar", sonarActualDistance);


        if(colorSensor.getRawLightDetected() >= ballLight){
            //if the distance is greater than a block then its a duck
            telemetry.addData("Detected", "Ball");
        } else {
            telemetry.addData("Detected", "Block");
        }
        boolean isTouch = ocSwitch.isTouch();

        telemetry.addData("Slide Switch Touched", isTouch);
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
            telemetry.update();
        }
        telemetry.addData("Stop", "Back");
        telemetry.update();
    }
}
