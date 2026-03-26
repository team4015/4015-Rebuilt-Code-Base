package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

/**
 * Shooter subsystem that manages flywheel/indexer coordination (with configurable delay) and motion-compensated aiming
 * at a fixed hood angle.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final PWMSparkMax shooterMotor = new PWMSparkMax(ShooterConstants.shooterMotorId);
    private final PWMSparkMax indexerMotor = new PWMSparkMax(ShooterConstants.indexerMotorId);

    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private boolean shooterActive = false;
    private boolean indexerActive = false;
    private Double pendingIndexerStartTimeSeconds = null;
    private double currentFlywheelOutput = ShooterConstants.shooterFullSpeed;
    private double currentIndexerOutput = ShooterConstants.indexerFullSpeed;

    /**
     * Creates the shooter subsystem.
     *
     * @param swerveSubsystem drivetrain used for motion compensation
     * @param limelightSubsystem vision subsystem used for target tracking
     */
    public ShooterSubsystem(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        shooterMotor.setInverted(ShooterConstants.shooterMotorInverted);
        indexerMotor.setInverted(ShooterConstants.indexerMotorInverted);
    }

    /** Toggles the flywheel between active and stopped states. */
    public void toggleShooter() {
        if (shooterActive || indexerActive || pendingIndexerStartTimeSeconds != null) {
            stopAll();
            return;
        }

        startShooterWithFollowerDelay();
    }

    /** Starts the flywheel at its configured full-speed output. */
    public void startShooter() {
        setShooterActive(true);
    }

    /** Starts the flywheel and schedules the indexer to follow after the configured delay. */
    public void startShooterWithFollowerDelay() {
        startShooter();
        scheduleIndexerFollower();
    }

    /** Schedules the indexer to start after the configured delay, or immediately when set to zero. */
    private void scheduleIndexerFollower() {
        double delaySeconds = Math.max(0.0, ShooterConstants.indexerStartDelaySeconds);
        if (delaySeconds <= 1e-4) {
            startIndexer();
            return;
        }

        pendingIndexerStartTimeSeconds = Timer.getFPGATimestamp() + delaySeconds;
    }

    /** Stops the flywheel. */
    public void stopShooter() {
        pendingIndexerStartTimeSeconds = null;
        setShooterActive(false);
    }

    /** Toggles the indexer between active and stopped states. */
    public void toggleIndexer() {
        if (indexerActive) {
            stopIndexer();
            return;
        }

        startIndexer();
    }

    /** Starts the indexer at its configured full-speed output. */
    public void startIndexer() {
        pendingIndexerStartTimeSeconds = null;
        setIndexerActive(true);
    }

    /** Stops the indexer. */
    public void stopIndexer() {
        setIndexerActive(false);
    }

    /** Stops both shooter and indexer outputs. */
    public void stopAll() {
        pendingIndexerStartTimeSeconds = null;
        stopShooter();
        stopIndexer();
    }

    /**
     * Returns whether the shooter is currently active.
     *
     * @return {@code true} when shooting is enabled
     */
    public boolean isShootingActive() {
        return shooterActive;
    }

    /**
     * Returns whether the indexer is currently active.
     *
     * @return {@code true} when the indexer is running
     */
    public boolean isIndexerActive() {
        return indexerActive;
    }

    /** Updates shot prediction and publishes shooter telemetry each scheduler cycle. */
    @Override
    public void periodic() {
        startIndexerAfterDelayIfReady();
        publishTelemetry();
    }

    /** Starts the indexer after the configured delay once the shooter is running. */
    private void startIndexerAfterDelayIfReady() {
        if (pendingIndexerStartTimeSeconds == null) {
            return;
        }

        if (!shooterActive) {
            pendingIndexerStartTimeSeconds = null;
            return;
        }

        if (Timer.getFPGATimestamp() >= pendingIndexerStartTimeSeconds) {
            startIndexer();
        }
    }

    private void setShooterActive(boolean active) {
        shooterActive = active;
        if (active) {
            shooterMotor.set(currentFlywheelOutput);
            return;
        }

        shooterMotor.stopMotor();
    }

    private void setIndexerActive(boolean active) {
        indexerActive = active;
        if (active) {
            indexerMotor.set(currentIndexerOutput);
            return;
        }

        indexerMotor.stopMotor();
    }

    /**
     * Publishes shooter state and latest ballistic solution values to SmartDashboard.
     *
     * <p>Displayed values include active state, fixed hood angle, yaw lead, flight time, and miss distance.
     */
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Shooter/Active", shooterActive);
        SmartDashboard.putBoolean("Shooter/IndexerActive", indexerActive);
        SmartDashboard.putNumber("Shooter/FlywheelSetpoint", currentFlywheelOutput);
        SmartDashboard.putNumber("Shooter/IndexerSetpoint", currentIndexerOutput);
    }

    /** Applies the nearest configured shot preset given distance and observed tag. */
    public void applyNearestPreset(double distanceMeters, int tagId) {
        Constants.ShooterConstants.HubFace face = faceForTag(tagId);
        double range = Double.isFinite(distanceMeters) ? distanceMeters : Double.POSITIVE_INFINITY;
        Constants.ShotPreset best = Constants.ShooterConstants.shotPresets.stream()
            .filter(p -> p.face == face)
            .min((a, b) -> Double.compare(
                Math.abs(a.rangeMeters - range),
                Math.abs(b.rangeMeters - range)))
            .orElseGet(() -> Constants.ShooterConstants.shotPresets.stream().findFirst().orElse(null));

        if (best == null) {
            // Fallback to defaults if no presets configured.
            currentFlywheelOutput = ShooterConstants.shooterFullSpeed;
            currentIndexerOutput = ShooterConstants.indexerFullSpeed;
            return;
        }

        currentFlywheelOutput = best.flywheel;
        currentIndexerOutput = best.indexer;
    }

    private Constants.ShooterConstants.HubFace faceForTag(int tagId) {
        if (Constants.ShooterConstants.leftTags.contains(tagId)) {
            return Constants.ShooterConstants.HubFace.LEFT;
        }
        if (Constants.ShooterConstants.rightTags.contains(tagId)) {
            return Constants.ShooterConstants.HubFace.RIGHT;
        }
        return Constants.ShooterConstants.HubFace.FRONT;
    }
}
