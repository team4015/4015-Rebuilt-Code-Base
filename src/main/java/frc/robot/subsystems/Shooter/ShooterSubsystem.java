package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
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
    private ChassisSpeeds previousRobotRelativeSpeeds = new ChassisSpeeds();
    private double previousMotionTimestampSeconds = Timer.getFPGATimestamp();
    private ShotCalculator.ShotSolution lastShotSolution = ShotCalculator.ShotSolution.noSolution();

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

    /**
     * Returns the yaw command needed to face the compensated shot solution while shooting.
     *
     * @return angular speed command in radians per second
     */
    public double getMotionCompensatedAimAngularSpeedRadPerSec() {
        if (!limelightSubsystem.hasValidTarget()) {
            return 0.0;
        }

        double correctedTxDegrees = limelightSubsystem.getTxDegrees() - Math.toDegrees(lastShotSolution.yawLeadRad);
        double omega = -correctedTxDegrees * frc.robot.Constants.VisionConstants.aimKp;
        return MathUtil.clamp(
            omega,
            -frc.robot.Constants.VisionConstants.aimMaxAngularSpeedRadPerSec,
            frc.robot.Constants.VisionConstants.aimMaxAngularSpeedRadPerSec
        );
    }

    /** Updates shot prediction and publishes shooter telemetry each scheduler cycle. */
    @Override
    public void periodic() {
        startIndexerAfterDelayIfReady();
        updateShotSolution();
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
            shooterMotor.set(ShooterConstants.shooterFullSpeed);
            return;
        }

        shooterMotor.stopMotor();
    }

    private void setIndexerActive(boolean active) {
        indexerActive = active;
        if (active) {
            indexerMotor.set(ShooterConstants.indexerFullSpeed);
            return;
        }

        indexerMotor.stopMotor();
    }

    /**
     * Recomputes the latest motion-compensated shot solution when shooting with a valid target.
     *
     * <p>The method estimates current robot acceleration from drivetrain velocity samples, predicts
     * control-latency-adjusted robot velocity, builds projectile inputs, and stores the newest
     * valid ballistic solution for aiming and telemetry.
     */
    private void updateShotSolution() {
        if (!shooterActive || !limelightSubsystem.hasValidTarget()) {
            return;
        }

        Translation2d hubCenterRobotSpace = limelightSubsystem.getHubCenterTranslationRobotSpace();
        if (!Double.isFinite(hubCenterRobotSpace.getX()) || !Double.isFinite(hubCenterRobotSpace.getY())) {
            return;
        }

        double nowSeconds = Timer.getFPGATimestamp();
        ChassisSpeeds currentRobotSpeeds = swerveSubsystem.getRobotRelativeSpeeds();
        double dt = Math.max(nowSeconds - previousMotionTimestampSeconds, 1e-3);
        Translation2d currentRobotVelocity = new Translation2d(
            currentRobotSpeeds.vxMetersPerSecond,
            currentRobotSpeeds.vyMetersPerSecond
        );
        Translation2d robotAcceleration = new Translation2d(
            (currentRobotSpeeds.vxMetersPerSecond - previousRobotRelativeSpeeds.vxMetersPerSecond) / dt,
            (currentRobotSpeeds.vyMetersPerSecond - previousRobotRelativeSpeeds.vyMetersPerSecond) / dt
        );
        Translation2d predictedRobotVelocity = currentRobotVelocity.plus(
            robotAcceleration.times(ShooterConstants.projectileControlLatencySeconds)
        );

        previousRobotRelativeSpeeds = currentRobotSpeeds;
        previousMotionTimestampSeconds = nowSeconds;

        double effectiveLaunchSpeed = ShooterConstants.shooterLaunchVelocityMetersPerSecond * ShooterConstants.projectileFlywheelBallFrictionCoefficient;
        double ballRadiusMeters = ShooterConstants.projectileBallDiameterMeters / 2.0;
        double crossSectionAreaMetersSquared = Math.PI * ballRadiusMeters * ballRadiusMeters;
        double fixedHoodAngleRad = ShooterConstants.hoodInitialAngleRadians;

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            hubCenterRobotSpace,
            predictedRobotVelocity,
            ShooterConstants.projectileReleaseHeightMeters,
            VisionConstants.hubCenterHeightMeters,
            effectiveLaunchSpeed,
            ShooterConstants.projectileBallMassKg,
            crossSectionAreaMetersSquared,
            ShooterConstants.projectileDragCoefficient,
            ShooterConstants.projectileAirDensityKgPerCubicMeter,
            ShooterConstants.projectileGravityMetersPerSecondSquared,
            fixedHoodAngleRad,
            fixedHoodAngleRad,
            ShooterConstants.projectileSolverTimeStepSeconds,
            ShooterConstants.projectileSolverMaxTimeSeconds
        );

        ShotCalculator.ShotSolution solution = ShotCalculator.solve(inputs);
        if (solution.valid) {
            lastShotSolution = solution;
        }
    }

    /**
     * Publishes shooter state and latest ballistic solution values to SmartDashboard.
     *
     * <p>Displayed values include active state, fixed hood angle, yaw lead, flight time, and miss distance.
     */
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Shooter/Active", shooterActive);
        SmartDashboard.putBoolean("Shooter/IndexerActive", indexerActive);
        SmartDashboard.putNumber("Shooter/FixedHoodAngleDeg", Math.toDegrees(ShooterConstants.hoodInitialAngleRadians));
        SmartDashboard.putNumber("Shooter/YawLeadDeg", Math.toDegrees(lastShotSolution.yawLeadRad));
        SmartDashboard.putNumber("Shooter/FlightTimeSec", lastShotSolution.flightTimeSeconds);
        SmartDashboard.putNumber("Shooter/MissDistanceM", lastShotSolution.missDistanceMeters);
    }
}
