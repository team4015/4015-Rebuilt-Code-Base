package frc.robot.config;

import com.google.gson.annotations.SerializedName;

/**
 * Typed representation of {@code driveConfig.json}.
 */
public class DriveConfig {
    public Module module;
    public Dimensions dimensions;
    public Motors motors;
    @SerializedName(value = "encoders", alternate = {"encoder"})
    public Encoders encoders;
    public Limits limits;
    public OI oi;

    /** Swerve module hardware ratios and steering gain. */
    public static class Module {
        public double wheelDiameterInches;
        public double driveGearRatio;
        public double turningGearRatio;
        @SerializedName(value = "turningKP", alternate = {"tuningKP"})
        public double turningKP;
    }

    /** Drivetrain geometry values. */
    public static class Dimensions {
        @SerializedName(value = "trackWidthInches", alternate = {"trachWidthInches"})
        public double trackWidthInches;
        public double wheelBaseInches;
    }

    /** Drive and steering motor IDs. */
    public static class Motors {
        public IntGroup drive;
        public IntGroup turning;
    }

    /** Absolute encoder ports, offsets, and inversion flags. */
    public static class Encoders {
        @SerializedName(value = "absolutePorts", alternate = {"absolutePort"})
        public IntGroup absolutePorts;
        @SerializedName(value = "absoluteOffsetsRad", alternate = {"absolutePortsOffsetRad", "absoluteOffsetRad"})
        public DoubleGroup absoluteOffsetsRad;
        public BooleanGroup driveReversed;
        public BooleanGroup turningReversed;
        public BooleanGroup absoluteReversed;
    }

    /** Integer values grouped by module corner. */
    public static class IntGroup {
        public int frontLeft;
        public int frontRight;
        public int backLeft;
        public int backRight;
    }

    /** Double values grouped by module corner. */
    public static class DoubleGroup {
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;
    }

    /** Boolean values grouped by module corner. */
    public static class BooleanGroup {
        public boolean frontLeft;
        public boolean frontRight;
        public boolean backLeft;
        public boolean backRight;
    }

    /** Drivetrain speed limits. */
    public static class Limits {
        @SerializedName(value = "maxSpeedMps", alternate = {"maxSpeed"})
        public double maxSpeedMps;
        public double maxAngularSpeedRadPerSec;
        public double teleopSpeedScale;
    }

    /** Controller bindings used by drive-related features. */
    public static class OI {
        public int driverControllerPort;
        public int driverYAxis;
        public int driverXAxis;
        public int driverRotAxis;
        public int shooterToggleButtonIdx;
        public int presetShootButtonIdx;
        public int driverFieldOrientedButtonIdx;
        public int aimAtTagButtonIdx;
        public double triggerPressedThreshold;
        public double deadband;
    }
}
