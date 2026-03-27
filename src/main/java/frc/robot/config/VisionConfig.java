package frc.robot.Config;

/**
 * Typed representation of {@code visionConfig.json}.
 */
public class VisionConfig {
    /** Limelight NetworkTables identity and camera mount values. */
    public Limelight limelight;
    /** Auto-aim tuning values. */
    public Aim aim;
    /** Hub target geometry used by the shot solver. */
    public Hub hub;

    /** Limelight-specific fields. */
    public static class Limelight {
        /** Limelight NetworkTables name (for example: "limelight"). */
        public String name;
        /** Lens height from floor in inches. */
        public double lensHeightInches;
        /** Upward camera mount angle in degrees. */
        public double mountAngleDegrees;
        /** Default AprilTag center height in inches for trig fallback distance. */
        public double defaultTagHeightInches;
    }

    /** Aiming behavior fields. */
    public static class Aim {
        /** Proportional gain from tx error (deg) to omega command (rad/s). */
        public double kp;
        /** Maximum absolute omega command while aiming. */
        public double maxAngularSpeedRadPerSec;
        /** Considered aligned when |tx| <= toleranceDegrees. */
        public double toleranceDegrees;
    }

    /** Hub geometry and AprilTag offset values. */
    public static class Hub {
        public double centerHeightMeters;
        public double openingWidthMeters;
        public double openingHeightMeters;
        public double frontEdgeToCenterMeters;
        public double aprilTagLateralOffsetMeters;
    }
}
