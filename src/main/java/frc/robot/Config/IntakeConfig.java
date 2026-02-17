package frc.robot.Config;

public class IntakeConfig {
    public GearRatio gearRatio;
    public Port port;
    public PID pid;

    public static class GearRatio{
        public double intake;
        public double extendableHopper;
    }

    public static class Port{
        public Motors intake;
        public Motors extendableHopper;

        public DigitalInput frontSwitch;
        public DigitalInput backSwitch;
    }

    public static class Motors{
        public int system;
    }

    public static class DigitalInput{
        public int limitSwitch;
    }

    public static class PID{
        public PIDValue intake;
        public PIDValue extendableHopper;
    }

    public static class PIDValue{
        public double p;
        public double i;
        public double d;
    }
}
