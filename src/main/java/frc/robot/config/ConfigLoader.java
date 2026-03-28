package frc.robot.Config;

import java.io.File;
import java.io.FileReader;
import java.io.Reader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Loads and validates JSON configuration files from the deploy directory.
 */
public final class ConfigLoader {
    private ConfigLoader() {
    }

    /**
     * Loads the drivetrain configuration.
     *
     * @return validated drive configuration
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
     * Loads the vision configuration.
     *
     * @return validated vision configuration
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
     * Loads the shooter configuration.
     *
     * @return validated shooter configuration
     */
    public static ShooterConfig loadShooterConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "shooterConfig.json");
            try (Reader reader = new FileReader(file)) {
                ShooterConfig config = new Gson().fromJson(reader, ShooterConfig.class);
                validateShooterConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load shooter config", e);
        }
    }

    /**
     * Loads the intake configuration.
     *
     * @return validated intake configuration
     */
    public static IntakeConfig loadIntakeConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "intakeConfig.json");
            try (Reader reader = new FileReader(file)) {
                IntakeConfig config = new Gson().fromJson(reader, IntakeConfig.class);
                validateIntakeConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load intake config", e);
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
            || config.limits == null
            || config.snapHeading == null
            || config.auto == null
            || config.oi == null) {
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
        requirePositive(config.limits.teleopMaxAccelUnitsPerSec, "limits.teleopMaxAccelUnitsPerSec");
        requirePositive(config.limits.teleopMaxAngularAccelUnitsPerSec, "limits.teleopMaxAngularAccelUnitsPerSec");

        requireNonNegative(config.snapHeading.kP, "snapHeading.kP");
        requirePositive(config.snapHeading.toleranceDegrees, "snapHeading.toleranceDegrees");

        if (config.auto.translationPid == null || config.auto.rotationPid == null) {
            throw new IllegalArgumentException("auto translationPid/rotationPid are required");
        }
        requireNonNegative(config.auto.translationPid.kP, "auto.translationPid.kP");
        requireNonNegative(config.auto.translationPid.kI, "auto.translationPid.kI");
        requireNonNegative(config.auto.translationPid.kD, "auto.translationPid.kD");
        requireNonNegative(config.auto.rotationPid.kP, "auto.rotationPid.kP");
        requireNonNegative(config.auto.rotationPid.kI, "auto.rotationPid.kI");
        requireNonNegative(config.auto.rotationPid.kD, "auto.rotationPid.kD");

        requireNonNegative(config.oi.triggerPressedThreshold, "oi.triggerPressedThreshold");
        requireNonNegative(config.oi.deadband, "oi.deadband");
        requireNonNegative(config.oi.presetShootButtonIdx, "oi.presetShootButtonIdx");
        requireNonNegative(config.oi.flopOverButtonIdx, "oi.flopOverButtonIdx");
        requireNonNegative(config.oi.extendIntakeToggleButtonIdx, "oi.extendIntakeToggleButtonIdx");
        requireNonNegative(config.oi.retractIntakeToggleButtonIdx, "oi.retractIntakeToggleButtonIdx");
    }

    private static void validateVisionConfig(VisionConfig config) {
        if (config == null
            || config.limelight == null
            || config.limelight.name == null
            || config.aim == null
            || config.hub == null) {
            throw new IllegalArgumentException("visionConfig.json is missing one or more required sections");
        }

        requirePositive(config.hub.centerHeightMeters, "hub.centerHeightMeters");
        requirePositive(config.hub.openingWidthMeters, "hub.openingWidthMeters");
        requirePositive(config.hub.openingHeightMeters, "hub.openingHeightMeters");
        requireNonNegative(config.hub.frontEdgeToCenterMeters, "hub.frontEdgeToCenterMeters");
    }


    private static void validateShooterConfig(ShooterConfig config) {
        if (config == null
            || config.motors == null
            || config.motors.flywheel == null
            || config.motors.indexer == null
            || config.flywheel == null
            || config.indexer == null) {
            throw new IllegalArgumentException("shooterConfig.json is missing one or more required sections");
        }


        if (config.presets == null) {
            config.presets = new ShooterConfig.ShotPresets();
        }
        if (config.presets.shots == null) {
            config.presets.shots = new ShooterConfig.Preset[0];
        }
        if (config.hubTags == null) {
            config.hubTags = new ShooterConfig.TagMapping();
        }
    }

    private static void validateIntakeConfig(IntakeConfig config) {
        if (config == null || config.motor == null || config.extendMotor == null) {
            throw new IllegalArgumentException("intakeConfig.json is missing one or more required sections");
        }

        requireNonNegative(config.motor.system, "motor.system");
        requireNonNegative(config.extendMotor.system, "extendMotor.system");
        requirePositive(config.fullSpeed, "fullSpeed");
        requireNonZero(config.extendOutSpeed, "extendOutSpeed");
        requireNonZero(config.retractSpeed, "retractSpeed");
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

    private static void requireNonZero(double value, String fieldName) {
        if (!Double.isFinite(value) || value == 0.0) {
            throw new IllegalArgumentException(fieldName + " must be a finite non-zero number");
        }
    }
}
