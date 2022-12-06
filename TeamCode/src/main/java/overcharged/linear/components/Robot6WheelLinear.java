package overcharged.linear.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.components.Drive;
import overcharged.components.Robot6Wheel;

/**
 * This class is for Autonomous
 */

public class Robot6WheelLinear
        extends Robot6Wheel
{
    protected LinearOpMode op;

    public Robot6WheelLinear(LinearOpMode op)
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
                odometryLocalization);
    }

    public TankDriveLinear getTankDriveLinear () {
        return (TankDriveLinear)this.drive;
    }

}
