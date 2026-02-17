package frc.robot.Config;

public class ExtendableHopperConfig {
    public IntakeConfig.GearRatio gearRatio;
    public IntakeConfig.Port port;
    public IntakeConfig.PID pid;

    public static class GearRatio{
        public double extendableHopper;
    }

    public static class Port{
        public IntakeConfig.Motors extendableHopper;

        public IntakeConfig.DigitalInput frontSwitch;
        public IntakeConfig.DigitalInput backSwitch;
    }

    public static class Motors{
        public int system;
    }

    public static class DigitalInput{
        public int limitSwitch;
    }

    public static class PID{ ;
        public IntakeConfig.PIDValue extendableHopper;
    }

    public static class PIDValue{
        public double p;
        public double i;
        public double d;
    }
}
