package frc.robot.Config;

import com.google.gson.annotations.SerializedName;

/**
 * Typed representation of {@code extendableHopper.json}.
 */
public class ExtendableHopperConfig {
    @SerializedName(value = "port", alternate = {"ports"})
    public Port port;
    public Settings settings;

    /** Hopper motor and limit-switch ports. */
    public static class Port{
        public IntakeConfig.Motors extendableHopper;

        public IntakeConfig.DigitalInput frontSwitch;
        public IntakeConfig.DigitalInput backSwitch;
    }

    /** Single motor assignment. */
    public static class Motors{
        public int system;
    }

    /** Single digital input assignment. */
    public static class DigitalInput{
        public int limitSwitch;
    }

    /** Hopper operating settings. */
    public static class Settings {
        public double matchStartForwardSpeed;
        public double manualSpeed;
    }
}
