package frc.robot.Config;

import com.google.gson.annotations.SerializedName;

public class ExtendableHopperConfig {
    public GearRatio gearRatio;
    @SerializedName(value = "port", alternate = {"ports"})
    public Port port;
    public PID pid;
    public Settings settings;

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

    public static class PID {
        public IntakeConfig.PIDValue extendableHopper;
    }

    public static class Settings {
        public double matchStartForwardSpeed;
        public double manualSpeed;
    }
}
