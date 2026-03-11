package frc.robot.Config;

import com.google.gson.annotations.SerializedName;

/**
 * Typed representation of {@code intakeConfig.json}.
 */
public class IntakeConfig {
    public Settings settings;

    /** Intake operating settings. */
    public static class Settings {
        public boolean intakeMotorReversed;
        public double fullSpeed;
    }
}
