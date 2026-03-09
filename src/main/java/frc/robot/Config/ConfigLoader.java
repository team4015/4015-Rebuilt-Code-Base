package frc.robot.Config;

import java.io.File;
import java.io.FileReader;
import java.io.Reader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.VisionConfig;

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

    public static IntakeConfig loadIntakeConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "intakeConfig.json");
            try (Reader reader = new FileReader(file)) {
                return new Gson().fromJson(reader, IntakeConfig.class);
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load intake config", e);
        }
    }

    public static ExtendableHopperConfig loadExtendableHopperConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "extendableHopper.json");
            try (Reader reader = new FileReader(file)) {
                return new Gson().fromJson(reader, ExtendableHopperConfig.class);
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load extendable hopper config", e);
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

    private static void validateVisionConfig(VisionConfig config) {
        if (config == null
            || config.limelight == null
            || config.limelight.name == null
            || config.aim == null) {
            throw new IllegalArgumentException("visionConfig.json is missing one or more required sections");
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
