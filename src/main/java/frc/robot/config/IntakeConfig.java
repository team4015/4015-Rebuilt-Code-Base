package frc.robot.Config;

/**
 * Typed representation of {@code intakeConfig.json}.
 */
public class IntakeConfig {
    public Motor motor;
    public Arm arm;
    public double fullSpeed;

    /** Generic single-motor assignment. */
    public static class Motor {
        public int system;
        public boolean inverted;
    }

    /** Intake arm deploy/retract motor and limit switch config. */
    public static class Arm {
        /** CAN ID for the arm SparkMax (brushed). */
        public int motorId;
        public boolean inverted;
        /** DIO channel for the lower hard-stop limit switch. */
        public int limitSwitchDio;
        /** Speed (0–1) used when driving the arm down. */
        public double downSpeed;
        /** Speed (0–1) used when driving the arm up. */
        public double upSpeed;
    }
}
