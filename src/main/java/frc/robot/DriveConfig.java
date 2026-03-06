package frc.robot;

import com.google.gson.annotations.SerializedName;

/**
 * Typed representation of {@code driveConfig.json}.
 *
 * <p>Fields are intentionally public for direct Gson binding and low-overhead constant extraction.
 * Alternate names are supported for backward compatibility with earlier JSON keys.</p>
 */
public class DriveConfig {
    /** Module geometry and gains. */
    public Module module;
    /** Chassis dimensions. */
    public Dimensions dimensions;
    /** CAN IDs for motors. */
    public Motors motors;
    /** Encoder channels, offsets, and inversion groups. */
    @SerializedName(value = "encoders", alternate = {"encoder"})
    public Encoders encoders;
    /** Speed and scaling limits. */
    public Limits limits;

    /** Module-level mechanical constants. */
    public static class Module {
        /** Wheel diameter in inches. */
        public double wheelDiameterInches;
        /** Drive ratio (motor rotations per wheel rotation). */
        public double driveGearRatio;
        /** Turning ratio (motor rotations per module rotation). */
        public double turningGearRatio;
        /** Steering proportional gain. */
        @SerializedName(value = "turningKP", alternate = {"tuningKP"})
        public double turningKP;
    }

    /** Drivetrain geometry dimensions in inches. */
    public static class Dimensions {
        /** Left-right wheel center distance in inches. */
        @SerializedName(value = "trackWidthInches", alternate = {"trachWidthInches"})
        public double trackWidthInches;
        /** Front-back wheel center distance in inches. */
        public double wheelBaseInches;
    }

    /** Motor CAN-ID groups for drive and turning motors. */
    public static class Motors {
        /** Drive motor IDs grouped by module corner. */
        public IntGroup drive;
        /** Turning motor IDs grouped by module corner. */
        public IntGroup turning;
    }

    /** Encoder channels, offsets, and inversion flags. */
    public static class Encoders {
        /** Absolute encoder analog channels grouped by module corner. */
        @SerializedName(value = "absolutePorts", alternate = {"absolutePort"})
        public IntGroup absolutePorts;
        /** Absolute encoder offsets in radians grouped by module corner. */
        @SerializedName(value = "absoluteOffsetsRad", alternate = {"absolutePortsOffsetRad", "absoluteOffsetRad"})
        public DoubleGroup absoluteOffsetsRad;
        /** Drive encoder inversion flags grouped by module corner. */
        public BooleanGroup driveReversed;
        /** Turning encoder inversion flags grouped by module corner. */
        public BooleanGroup turningReversed;
        /** Absolute encoder reversal flags grouped by module corner. */
        public BooleanGroup absoluteReversed;
    }

    /** Common 4-corner integer grouping (FL/FR/BL/BR). */
    public static class IntGroup {
        /** Front-left value. */
        public int frontLeft;
        /** Front-right value. */
        public int frontRight;
        /** Back-left value. */
        public int backLeft;
        /** Back-right value. */
        public int backRight;
    }

    /** Common 4-corner double grouping (FL/FR/BL/BR). */
    public static class DoubleGroup {
        /** Front-left value. */
        public double frontLeft;
        /** Front-right value. */
        public double frontRight;
        /** Back-left value. */
        public double backLeft;
        /** Back-right value. */
        public double backRight;
    }

    /** Common 4-corner boolean grouping (FL/FR/BL/BR). */
    public static class BooleanGroup {
        /** Front-left value. */
        public boolean frontLeft;
        /** Front-right value. */
        public boolean frontRight;
        /** Back-left value. */
        public boolean backLeft;
        /** Back-right value. */
        public boolean backRight;
    }

    /** Drivetrain speed limits and teleop scaling. */
    public static class Limits {
        /** Maximum translational speed in meters per second. */
        @SerializedName(value = "maxSpeedMps", alternate = {"maxSpeed"})
        public double maxSpeedMps;
        /** Maximum angular speed in radians per second. */
        public double maxAngularSpeedRadPerSec;
        /** Teleop scaling factor in range [0, 1]. */
        public double teleopSpeedScale;
    }
}
