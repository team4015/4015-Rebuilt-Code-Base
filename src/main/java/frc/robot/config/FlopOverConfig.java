package frc.robot.Config;

/**
 * Typed representation of {@code flopOverConfig.json}.
 */
public class FlopOverConfig {
    public Motor motor;
    public double deployDownSpeed;
    public LimitSwitch limitSwitch;

    /** Generic single-motor assignment. */
    public static class Motor {
        public int system;
        public boolean inverted;
    }

    /** Downward travel limit switch configuration. */
    public static class LimitSwitch {
        public int dioPort;
        /** When true, a high signal means the switch is pressed; when false (default) low means pressed. */
        public boolean activeHigh;
    }
}
