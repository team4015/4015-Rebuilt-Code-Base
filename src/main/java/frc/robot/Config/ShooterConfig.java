package frc.robot.Config;

/**
 * Typed representation of {@code shooterConfig.json}.
 */
public class ShooterConfig {
    public Motors motors;
    public Limits limits;
    public Flywheel flywheel;
    public Indexer indexer;
    public Hood hood;
    public Projectile projectile;

    /** Shooter-related motor assignments. */
    public static class Motors {
        public FlywheelMotors flywheel;
        public Motor indexer;
        public Motor hood;
    }

    /** Dual-flywheel motor IDs and inversion flags. */
    public static class FlywheelMotors {
        public int leader;
        public int follower;
        public boolean leaderInverted;
        public boolean followerInverted;
    }

    /** Generic single-motor assignment. */
    public static class Motor {
        public int system;
        public boolean inverted;
    }

    /** Shooter limit-switch assignments. */
    public static class Limits {
        public int hoodMinLimitSwitch;
        public int hoodMaxLimitSwitch;
    }

    /** Flywheel operating values. */
    public static class Flywheel {
        public double fullSpeed;
        public double launchVelocityMetersPerSecond;
    }

    /** Indexer operating values. */
    public static class Indexer {
        public double fullSpeed;
    }

    /** Hood mechanical and control values. */
    public static class Hood {
        public double gearRatio;
        public double initialAngleDegrees;
        public double maxAngleDegrees;
        public double homingOutput;
        public double maxControlOutput;
        public double toleranceDegrees;
        public PID pid;
    }

    /** PID gains for the hood controller. */
    public static class PID {
        public double p;
        public double i;
        public double d;
    }

    /** Projectile-model values used by the shot solver. */
    public static class Projectile {
        public double releaseHeightMeters;
        public double ballMassKg;
        public double ballDiameterMeters;
        public double dragCoefficient;
        public double airDensityKgPerCubicMeter;
        public double flywheelBallFrictionCoefficient;
        public double gravityMetersPerSecondSquared;
        public double solverTimeStepSeconds;
        public double solverMaxTimeSeconds;
        public double controlLatencySeconds;
    }
}
