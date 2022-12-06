package overcharged.linear.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.components.Drive;
import overcharged.components.Robot6WheelTest2;

/**
 * This class is for Autonomous
 */

public class Robot6WheelTest2Linear
        extends Robot6WheelTest2 {
    protected LinearOpMode op;

    public Robot6WheelTest2Linear(LinearOpMode op) {
        super(op, true);
        this.op = op;
        getTankDriveLinearTest2().setLinearOpMode(op);
    }

    @Override
    protected Drive createDrive() {
        return new TankDriveLinearTest2(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                gyroSensor,
                left, right,
                null);
    }

//    public TankDriveLinear getTankDriveLinear() {
//        return (TankDriveLinear) this.drive;
//    }

    public TankDriveLinearTest2 getTankDriveLinearTest2() {
        return (TankDriveLinearTest2) this.drive;
    }

}
