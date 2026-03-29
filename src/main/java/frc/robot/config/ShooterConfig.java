package frc.robot.Config;

/**
 * Typed representation of {@code shooterConfig.json}.
 */
public class ShooterConfig {
    public Motors motors;
    public Flywheel flywheel;
    public Indexer indexer;
    public ShotPresets presets;
    public TagMapping hubTags;

    /** Shooter-related motor assignments. */
    public static class Motors {
        public FlywheelMotors flywheel;
        public Motor indexer;
    }

    /** Dual-flywheel motor IDs and inversion flags. */
    public static class FlywheelMotors {
        public int leader;
        public boolean leaderInverted;
    }

    /** Generic single-motor assignment. */
    public static class Motor {
        public int system;
        public boolean inverted;
    }

    /** Flywheel operating values. */
    public static class Flywheel {
        public double fullSpeed;
        public double launchVelocityMetersPerSecond;
    }

    /** Indexer operating values. */
    public static class Indexer {
        public double fullSpeed;
        public double startDelaySeconds;
    }

    /** Optional table of preset shots keyed by hub face. */
    public static class ShotPresets {
        public Preset[] shots;
    }

    /** One preset entry. */
    public static class Preset {
        public String face;          // "front", "left", "right"
        public double rangeMeters;   // distance from hub center
        public double flywheel;      // motor output [-1,1] (or use RPM if you change control mode)
        public double indexer;       // motor output [-1,1]
    }

    /** Mapping from AprilTag IDs to hub faces (supports both alliances). */
    public static class TagMapping {
        public int[] front;
        public int[] left;
        public int[] right;
        public int[] frontBlue;
        public int[] leftBlue;
        public int[] rightBlue;
    }
}
