package overcharged.linear.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.components.Drive;
import overcharged.components.Robot6WheelTest;

/**
 * This class is for Autonomous
 */

public class Robot6WheelTestLinear
        extends Robot6WheelTest
{
    protected LinearOpMode op;

    public Robot6WheelTestLinear(LinearOpMode op)
    {
        super(op,true);
        this.op = op;
        getTankDriveLinear().setLinearOpMode(op);
    }

    @Override
    protected Drive createDrive() {
        return new TankDriveLinear(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                gyroSensor,
                null);
    }

    public TankDriveLinear getTankDriveLinear () {
        return (TankDriveLinear)this.drive;
    }

}
