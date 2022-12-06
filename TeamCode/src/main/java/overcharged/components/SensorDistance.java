package overcharged.components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import overcharged.linear.util.WaitLinear;

public class SensorDistance {
    Rev2mDistanceSensor sensorDistance;

    public SensorDistance(HardwareMap hardwareMap, String name) {
        sensorDistance = hardwareMap.get(Rev2mDistanceSensor.class, name);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorDistance;

    }

    public double getDistance(){
        double distance = sensorDistance.getDistance(DistanceUnit.INCH);
        return distance;
    }



    /**
     * @param low distance lower bound
     * @param high distance higher bound
     * @return same as getDistance
     * Tries to poll distance multiple times and repolls if distance > high || distance < low
     */
    public double slowPoll(WaitLinear wait, int low, int high) throws InterruptedException{
        int pollNum = 0;
        do{
            double distance = getDistance();
            if(distance > low && distance < high){
                return distance;
            }
            wait.waitMillis(150);
            pollNum++;
        }while(pollNum < 3);
        return -1;

    }
}
