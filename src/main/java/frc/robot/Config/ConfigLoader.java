package frc.robot.Config;

import java.io.File;
import java.io.FileReader;
import java.io.Reader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;

public final class ConfigLoader {
    private ConfigLoader() {
    }

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

        requirePositive(config.module.wheelDiameterInches, "module.wheelDiameterInches");
        requirePositive(config.module.driveGearRatio, "module.driveGearRatio");
        requirePositive(config.module.turningGearRatio, "module.turningGearRatio");
        requireNonNegative(config.module.turningKP, "module.turningKP");
        requirePositive(config.dimensions.trackWidthInches, "dimensions.trackWidthInches");
        requirePositive(config.dimensions.wheelBaseInches, "dimensions.wheelBaseInches");
        requirePositive(config.limits.maxSpeedMps, "limits.maxSpeedMps");
        requirePositive(config.limits.maxAngularSpeedRadPerSec, "limits.maxAngularSpeedRadPerSec");

        if (config.limits.teleopSpeedScale < 0.0 || config.limits.teleopSpeedScale > 1.0) {
            throw new IllegalArgumentException("limits.teleopSpeedScale must be in the range [0, 1]");
        }
    }
    private static void requirePositive(double value, String fieldName) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(fieldName + " must be a finite positive number");
        }
    }

    private static void requireNonNegative(double value, String fieldName) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(fieldName + " must be a finite non-negative number");
        }
    }
}
