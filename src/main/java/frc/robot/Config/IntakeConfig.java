package frc.robot.Config;

import com.google.gson.annotations.SerializedName;

/**
 * Typed representation of {@code intakeConfig.json}.
 */
public class IntakeConfig {
    @SerializedName(value = "port", alternate = {"ports"})
    public Port port;
    public Settings settings;

    /** Intake motor ports. */
    public static class Port{
        public Motors intake;
    }

    /** Single motor assignment. */
    public static class Motors{
        public int system;
    }

    /** Single digital input assignment. */
    public static class DigitalInput{
        public int limitSwitch;
    }

    /** Intake operating settings. */
    public static class Settings {
        public boolean intakeMotorReversed;
        public double fullSpeed;
    }
}
