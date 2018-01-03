
/**
 * This is a standard digital PID controller. It also has anti-windup features.
 * @author Henrik Söderlund on 12/23/2017.
 */
public class PID {
    // PID Parameters
    private double kp = 1;
    private double ki = 0;
    private double kd = 0;

    // Anti-windup
    private boolean antiWindup = false;

    // Parameters for computing control signal
    private double errorSum = 0;
    private double prevError = 0;
    private double lastTimeMillis = 0;
    private double prevOutput = 0;

    // Saturation limits used for anti-windup function.
    private double minimum = Integer.MIN_VALUE;
    private double maximum = Integer.MAX_VALUE;

    /**
     * Constructor of PID object
     * @param kp P-parameter
     * @param ki I-parameter
     * @param kd D-parameter
     */
    public PID (double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Sets the anti-windup functionality on or off.
     * This functionality is only useful if you have the I-parameter
     * non-zero and your setpoint is close to one of the saturation limits.
     * @param active True for on, False for off
     */
    public void setAntiWindupMode (boolean active) {
        antiWindup = active;
    }

    /**
     * Sets the satration limits for anti-windup functionality.
     * @param minimum Minimum control signal value
     * @param maximum Maximum control signal value
     */
    public void setSaturationLimits (double minimum, double maximum) {
        this.minimum = minimum;
        this.maximum = maximum;
    }

    /**
     * The method for computing the output control signa based on the input error.
     * The method follows the standard parallel PID equation.
     * @param error Error signal
     * @return Control signal
     */
    public double compute (double error) {
        double currTime = ((double)System.nanoTime())/10000.0;

        if (lastTimeMillis == 0)
            lastTimeMillis = currTime;

        double dt = (currTime - lastTimeMillis)/10000.0;
        double errorDeriv = dt > 0 ? (error - prevError)/dt : 0;

        // This is where anti-windup functionality takes effect.
        if (antiWindup == false || (prevOutput < maximum && error > 0) && (prevOutput > minimum && error < 0))
            errorSum += error*dt;

        prevError = error;

        // Combine into output control signal
        double output = kp*error + ki*errorSum + kd*errorDeriv;

        prevOutput = output;

        lastTimeMillis = currTime;
        return output;
    }
}
