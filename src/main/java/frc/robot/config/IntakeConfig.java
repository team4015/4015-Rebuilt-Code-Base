package frc.robot.Config;

/**
 * Typed representation of {@code intakeConfig.json}.
 */
public class IntakeConfig {
    public Motor motor;
    public Motor extendMotor;
    public double fullSpeed;
    public double extendOutSpeed;
    public double retractSpeed;

    /** Generic single-motor assignment. */
    public static class Motor {
        public int system;
        public boolean inverted;
    }
}
