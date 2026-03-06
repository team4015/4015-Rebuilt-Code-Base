package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.io.Reader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Utility for loading and validating drivetrain configuration from deploy JSON.
 */
public final class ConfigLoader {
    private ConfigLoader() {
    }

    /**
     * Loads {@code driveConfig.json} from the robot deploy directory.
     *
     * @return parsed and validated drive configuration
     * @throws RuntimeException if file read/parse/validation fails
     */
    public static DriveConfig loadDriveConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "driveConfig.json");
            try (Reader reader = new FileReader(file)) {
                DriveConfig config = new Gson().fromJson(reader, DriveConfig.class);
                validateConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load drive config", e);
        }
    }

    /**
     * Validates required sections of the drive config before constants are consumed.
     *
     * @param config parsed config object
     * @throws IllegalArgumentException if any required section is missing
     */
    private static void validateConfig(DriveConfig config) {
        if (config == null
            || config.module == null
            || config.dimensions == null
            || config.motors == null
            || config.motors.drive == null
            || config.motors.turning == null
            || config.encoders == null
            || config.encoders.absolutePorts == null
            || config.encoders.absoluteOffsetsRad == null
            || config.encoders.driveReversed == null
            || config.encoders.turningReversed == null
            || config.encoders.absoluteReversed == null
            || config.limits == null) {
            throw new IllegalArgumentException("driveConfig.json is missing one or more required sections");
        }
    }
}
