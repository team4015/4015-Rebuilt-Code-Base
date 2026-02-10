package frc.robot;

public class DriveConfig {
    public Module module;
    public Dimensions dimensions;
    public Motors motors;
    public Encoders encoders;
    public Limits limits;

    public static class Module{
        public double wheelDiameterInches;
        public double driveGearRatio;
        public double turningGearRatio;
        public double turningKP;
    }

    public static class Dimensions{
        public double trachWidthInches;
        public double wheelBaseInches;
    }

    public static class Motors{
        public MotorGroup drive;
        public MotorGroup turning;
    }

    public static class MotorGroup{
        public int frontLeft;
        public int frontRight;
        public int backLeft;
        public int backRight;
    }

    public static class Encoders{
        public MotorGroup absolutePort;
        public MotorGroup absoluteOffsetsRad;
        public MotorGroup driveReversed;
        public MotorGroup turningReversed;
    }
    
    public static class Limits{
        public double maxSpeedMps;
        public double maxAngularSpeedRadPerSec;
        public double teleopSpeedScale;
    }
}
