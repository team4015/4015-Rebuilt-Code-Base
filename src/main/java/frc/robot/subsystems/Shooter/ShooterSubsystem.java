package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

/**
 * Shooter subsystem that manages flywheels, indexer, hood control, and motion-compensated aiming.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterLeaderMotor =
        new SparkMax(ShooterConstants.shooterLeaderMotorId, MotorType.kBrushless);
    private final SparkMax shooterFollowerMotor =
        new SparkMax(ShooterConstants.shooterFollowerMotorId, MotorType.kBrushless);
    private final SparkMax indexerMotor =
        new SparkMax(ShooterConstants.indexerMotorId, MotorType.kBrushless);
    private final SparkMax hoodMotor =
        new SparkMax(ShooterConstants.hoodMotorId, MotorType.kBrushless);

    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final DigitalInput hoodMinLimitSwitch = new DigitalInput(ShooterConstants.hoodMinLimitSwitchPort);
    private final DigitalInput hoodMaxLimitSwitch = new DigitalInput(ShooterConstants.hoodMaxLimitSwitchPort);
    private final PIDController hoodPid = new PIDController(
        ShooterConstants.hoodKp,
        ShooterConstants.hoodKi,
        ShooterConstants.hoodKd
    );

    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private boolean shootingActive = false;
    private boolean hoodResetting = false;
    private double targetHoodAngleRad = ShooterConstants.hoodInitialAngleRadians;
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

        configureMotor(shooterLeaderMotor, ShooterConstants.shooterLeaderMotorInverted, IdleMode.kCoast);
        configureMotor(shooterFollowerMotor, ShooterConstants.shooterFollowerMotorInverted, IdleMode.kCoast);
        configureMotor(indexerMotor, ShooterConstants.indexerMotorInverted, IdleMode.kBrake);
        configureHoodMotor();

        hoodPid.setTolerance(ShooterConstants.hoodToleranceRadians);
        hoodEncoder.setPosition(0.0);
    }

    private void configureMotor(SparkMax motor, boolean inverted, IdleMode idleMode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted).idleMode(idleMode);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureHoodMotor() {
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.inverted(ShooterConstants.hoodMotorInverted).idleMode(IdleMode.kBrake);
        hoodConfig.encoder
            .positionConversionFactor((2.0 * Math.PI) / ShooterConstants.hoodGearRatio)
            .velocityConversionFactor(((2.0 * Math.PI) / ShooterConstants.hoodGearRatio) / 60.0);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Toggles the shooter and indexer between active and stopped states. */
    public void toggleShooting() {
        if (shootingActive) {
            stopShooting();
            return;
        }

        startShooting();
    }

    /** Starts both flywheels and the indexer at their configured full-speed outputs. */
    public void startShooting() {
        shootingActive = true;
        shooterLeaderMotor.set(ShooterConstants.shooterFullSpeed);
        shooterFollowerMotor.set(ShooterConstants.shooterFullSpeed);
        indexerMotor.set(ShooterConstants.indexerFullSpeed);
    }

    /** Stops the flywheels and indexer. */
    public void stopShooting() {
        shootingActive = false;
        shooterLeaderMotor.stopMotor();
        shooterFollowerMotor.stopMotor();
        indexerMotor.stopMotor();
    }

    /**
     * Returns whether the shooter is currently active.
     *
     * @return {@code true} when shooting is enabled
     */
    public boolean isShootingActive() {
        return shootingActive;
    }

    /** Starts the hood homing routine using the minimum-angle limit switch. */
    public void beginHoodReset() {
        hoodResetting = true;
        stopShooting();
    }

    /**
     * Returns whether the hood is currently executing its reset routine.
     *
     * @return {@code true} when the hood is homing
     */
    public boolean isHoodResetting() {
        return hoodResetting;
    }

    /**
     * Returns whether the hood PID controller is currently within tolerance.
     *
     * @return {@code true} when the hood is at its target angle
     */
    public boolean isHoodAtTarget() {
        return hoodPid.atSetpoint();
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

    @Override
    public void periodic() {
        if (hoodMinLimitSwitch.get()) {
            hoodEncoder.setPosition(0.0);
        }

        if (hoodResetting) {
            runHoodResetRoutine();
            publishTelemetry();
            return;
        }

        updateShotSolution();
        runHoodClosedLoop();
        publishTelemetry();
    }

    private void updateShotSolution() {
        if (!shootingActive || !limelightSubsystem.hasValidTarget()) {
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

        double effectiveLaunchSpeed = ShooterConstants.shooterLaunchVelocityMetersPerSecond
            * ShooterConstants.projectileFlywheelBallFrictionCoefficient;
        double ballRadiusMeters = ShooterConstants.projectileBallDiameterMeters / 2.0;
        double crossSectionAreaMetersSquared = Math.PI * ballRadiusMeters * ballRadiusMeters;

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
            ShooterConstants.hoodInitialAngleRadians,
            ShooterConstants.hoodMaxAngleRadians,
            ShooterConstants.projectileSolverTimeStepSeconds,
            ShooterConstants.projectileSolverMaxTimeSeconds
        );

        ShotCalculator.ShotSolution solution = ShotCalculator.solve(inputs);
        if (solution.valid) {
            lastShotSolution = solution;
            targetHoodAngleRad = MathUtil.clamp(
                solution.hoodAngleRad,
                ShooterConstants.hoodInitialAngleRadians,
                ShooterConstants.hoodMaxAngleRadians
            );
        }
    }

    private void runHoodClosedLoop() {
        double currentAngleRad = getCurrentHoodAngleRad();
        double output = MathUtil.clamp(
            hoodPid.calculate(currentAngleRad, targetHoodAngleRad),
            -ShooterConstants.hoodMaxControlOutput,
            ShooterConstants.hoodMaxControlOutput
        );

        if (hoodMinLimitSwitch.get() && output < 0.0) {
            output = 0.0;
        }

        if (hoodMaxLimitSwitch.get() && output > 0.0) {
            output = 0.0;
        }

        hoodMotor.set(output);
    }

    private void runHoodResetRoutine() {
        if (hoodMinLimitSwitch.get()) {
            hoodMotor.stopMotor();
            hoodEncoder.setPosition(0.0);
            targetHoodAngleRad = ShooterConstants.hoodInitialAngleRadians;
            hoodResetting = false;
            return;
        }

        hoodMotor.set(ShooterConstants.hoodHomingOutput);
    }

    private double getCurrentHoodAngleRad() {
        return ShooterConstants.hoodInitialAngleRadians + hoodEncoder.getPosition();
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Shooter/Active", shootingActive);
        SmartDashboard.putBoolean("Shooter/HoodResetting", hoodResetting);
        SmartDashboard.putBoolean("Shooter/HoodMinLimit", hoodMinLimitSwitch.get());
        SmartDashboard.putBoolean("Shooter/HoodMaxLimit", hoodMaxLimitSwitch.get());
        SmartDashboard.putNumber("Shooter/HoodAngleDeg", Math.toDegrees(getCurrentHoodAngleRad()));
        SmartDashboard.putNumber("Shooter/HoodTargetDeg", Math.toDegrees(targetHoodAngleRad));
        SmartDashboard.putNumber("Shooter/YawLeadDeg", Math.toDegrees(lastShotSolution.yawLeadRad));
        SmartDashboard.putNumber("Shooter/FlightTimeSec", lastShotSolution.flightTimeSeconds);
        SmartDashboard.putNumber("Shooter/MissDistanceM", lastShotSolution.missDistanceMeters);
    }
}
