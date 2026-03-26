package frc.robot.config;

/**
 * Typed representation of {@code flopOverConfig.json}.
 */
public class FlopOverConfig {
    public Motor motor;
    public LimitSwitches limits;
    /**
     * Absolute PWM outputs used when driving clockwise (positive) and counter-clockwise (negative).
     * Values should be in the range [0, 1].
     */
    public double clockwiseOutput;
    public double counterClockwiseOutput;
    /** Whether to begin motion in the clockwise direction when toggled on. */
    public boolean startClockwise;

    /** PWM motor assignment and inversion for the flop-over mechanism. */
    public static class Motor {
        public int pwmPort;
        public boolean inverted;
    }

    /** Digital-input channel assignments for the two travel limit switches. */
    public static class LimitSwitches {
        public int forwardDio;
        public int backwardDio;
        /**
         * If true, {@code DigitalInput.get()} returning false indicates a pressed switch (typical
         * with the roboRIO's pull-up); if false, true indicates pressed.
         */
        public boolean normallyClosed;
    }
}
