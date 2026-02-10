package frc.robot;

public class DriveConfig {
    public Module module;
    public Dimensions dimensions;
    public Motors motors;
    public Encoders encoders;
    public Limits limits;

    public static class Module {
        public double wheelDiameterInches;
        public double driveGearRatio;
        public double turningGearRatio;
        public double turningKP;
    }

    public static class Dimensions {
        public double trackWidthInches;
        public double wheelBaseInches;
    }

    public static class Motors {
        public MotorGroup drive;
        public MotorGroup turning;
    }

    public static class MotorGroup {
        public int frontLeft;
        public int frontRight;
        public int backLeft;
        public int backRight;
    }

    public static class Encoders {
        public MotorGroup absolutePorts;
        public DoubleGroup absoluteOffsetsRad;
        public BooleanGroup driveReversed;
        public BooleanGroup turningReversed;
        public BooleanGroup absoluteReversed;
    }

    public static class DoubleGroup {
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;
    }

    public static class BooleanGroup {
        public boolean frontLeft;
        public boolean frontRight;
        public boolean backLeft;
        public boolean backRight;
    }

    public static class Limits {
        public double maxSpeedMps;
        public double maxAngularSpeedRadPerSec;
        public double teleopSpeedScale;
    }
}
