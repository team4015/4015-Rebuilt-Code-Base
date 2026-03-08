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
                validateDriveConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load drive config", e);
        }
    }

    /**
     * Loads {@code visionConfig.json} from the robot deploy directory.
     *
     * @return parsed and validated vision configuration
     * @throws RuntimeException if file read/parse/validation fails
     */
    public static VisionConfig loadVisionConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "visionConfig.json");
            try (Reader reader = new FileReader(file)) {
                VisionConfig config = new Gson().fromJson(reader, VisionConfig.class);
                validateVisionConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load vision config", e);
        }
    }

    /**
     * Validates required sections of the drive config before constants are consumed.
     *
     * @param config parsed config object
     * @throws IllegalArgumentException if any required section is missing
     */
    private static void validateDriveConfig(DriveConfig config) {
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

    /**
     * Validates required sections of the vision config.
     *
     * @param config parsed config object
     * @throws IllegalArgumentException if any required section is missing
     */
    private static void validateVisionConfig(VisionConfig config) {
        if (config == null
            || config.limelight == null
            || config.limelight.name == null
            || config.aim == null) {
            throw new IllegalArgumentException("visionConfig.json is missing one or more required sections");
        }
    }
}
