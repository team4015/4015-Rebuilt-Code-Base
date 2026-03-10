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

    /**
     * Loads the hopper configuration.
     *
     * @return validated hopper configuration
     */
    public static ExtendableHopperConfig loadExtendableHopperConfig() {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "extendableHopper.json");
            try (Reader reader = new FileReader(file)) {
                ExtendableHopperConfig config = new Gson().fromJson(reader, ExtendableHopperConfig.class);
                validateExtendableHopperConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load extendable hopper config", e);
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

        requireNonNegative(config.oi.triggerPressedThreshold, "oi.triggerPressedThreshold");
        requireNonNegative(config.oi.deadband, "oi.deadband");
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

    private static void validateIntakeConfig(IntakeConfig config) {
        if (config == null
            || config.port == null
            || config.port.intake == null
            || config.settings == null) {
            throw new IllegalArgumentException("intakeConfig.json is missing one or more required sections");
        }

        requireNonNegative(config.settings.fullSpeed, "settings.fullSpeed");
    }

    private static void validateExtendableHopperConfig(ExtendableHopperConfig config) {
        if (config == null
            || config.port == null
            || config.port.extendableHopper == null
            || config.port.frontSwitch == null
            || config.port.backSwitch == null
            || config.settings == null) {
            throw new IllegalArgumentException("extendableHopper.json is missing one or more required sections");
        }

        requireNonNegative(config.settings.matchStartForwardSpeed, "settings.matchStartForwardSpeed");
        requireNonNegative(config.settings.manualSpeed, "settings.manualSpeed");
    }

    private static void validateShooterConfig(ShooterConfig config) {
        if (config == null
            || config.motors == null
            || config.motors.flywheel == null
            || config.motors.indexer == null
            || config.motors.hood == null
            || config.limits == null
            || config.flywheel == null
            || config.indexer == null
            || config.hood == null
            || config.hood.pid == null
            || config.projectile == null) {
            throw new IllegalArgumentException("shooterConfig.json is missing one or more required sections");
        }

        requirePositive(config.flywheel.fullSpeed, "flywheel.fullSpeed");
        requirePositive(config.flywheel.launchVelocityMetersPerSecond, "flywheel.launchVelocityMetersPerSecond");
        requirePositive(config.indexer.fullSpeed, "indexer.fullSpeed");
        requirePositive(config.hood.gearRatio, "hood.gearRatio");
        requirePositive(config.hood.maxAngleDegrees, "hood.maxAngleDegrees");
        requirePositive(config.hood.maxControlOutput, "hood.maxControlOutput");
        requirePositive(config.hood.toleranceDegrees, "hood.toleranceDegrees");
        requirePositive(config.projectile.releaseHeightMeters, "projectile.releaseHeightMeters");
        requirePositive(config.projectile.ballMassKg, "projectile.ballMassKg");
        requirePositive(config.projectile.ballDiameterMeters, "projectile.ballDiameterMeters");
        requirePositive(config.projectile.dragCoefficient, "projectile.dragCoefficient");
        requirePositive(config.projectile.airDensityKgPerCubicMeter, "projectile.airDensityKgPerCubicMeter");
        requirePositive(config.projectile.flywheelBallFrictionCoefficient, "projectile.flywheelBallFrictionCoefficient");
        requirePositive(config.projectile.gravityMetersPerSecondSquared, "projectile.gravityMetersPerSecondSquared");
        requirePositive(config.projectile.solverTimeStepSeconds, "projectile.solverTimeStepSeconds");
        requirePositive(config.projectile.solverMaxTimeSeconds, "projectile.solverMaxTimeSeconds");
        requireNonNegative(config.projectile.controlLatencySeconds, "projectile.controlLatencySeconds");
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
