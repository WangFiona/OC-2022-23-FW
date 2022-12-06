package overcharged.util;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Advaith Nair on 9/24/2019.
 */
public class PIDCalculator {
    private double kP, kI, kD, constant = 0;
    private double lastError = 0;
    private double integral = 0;

    /**
     * Set parameters for the PID calculator. Must set the parameters every loop.
     *
     * @param kP       proportional coefficient
     * @param kI       integral coefficient
     * @param kD       derivative coefficient
     * @param constant constant value to add to the PID result
     */
    public PIDCalculator(double kP, double kI, double kD, double constant) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.constant = constant;
    }

    /**
     * Set parameters for the PID calculator. Must set the parameters every loop.
     *
     * @param kP proportional coefficient
     * @param kI integral coefficient
     * @param kD derivative coefficient
     */
    public PIDCalculator(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    /**
     * @return the result of the PID calculations
     */
    public double getPID(double error) {
        double derivative = error - lastError;
        integral += error;
        lastError = error;

        return Range.clip((kP * error) + (kI * integral) + (kD * derivative) + constant, -1.0, 1.0);
    }
}