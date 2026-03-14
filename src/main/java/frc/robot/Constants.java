package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Config.ConfigLoader;
import frc.robot.Config.DriveConfig;
import frc.robot.Config.ShooterConfig;
import frc.robot.Config.VisionConfig;

/**
 * Central access point for typed, JSON-backed robot constants.
 */
public final class Constants {
    public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
    public static final ShooterConfig SHOOTER = ConfigLoader.loadShooterConfig();
    public static final VisionConfig VISION = ConfigLoader.loadVisionConfig();

    private Constants() {
    }

    /** Constants used by individual swerve modules. */
    public static final class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);
        public static final double driveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;
        public static final double turningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;
        public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;
        public static final double turningEncoderRot2Rad = turningMotorGearRatio * 2.0 * Math.PI;
        public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60.0;
        public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad / 60.0;
        public static final double kPTurning = DRIVE.module.turningKP;

        private ModuleConstants() {
        }
    }

    /** Constants used by the swerve drivetrain as a whole. */
    public static final class DriveConstants {
        public static final double trackWidth = Units.inchesToMeters(DRIVE.dimensions.trackWidthInches);
        public static final double wheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

        public static final SwerveDriveKinematics driveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );

        public static final int frontLeftDriveMotorPort = DRIVE.motors.drive.frontLeft;
        public static final int frontRightDriveMotorPort = DRIVE.motors.drive.frontRight;
        public static final int backLeftDriveMotorPort = DRIVE.motors.drive.backLeft;
        public static final int backRightDriveMotorPort = DRIVE.motors.drive.backRight;

        public static final int frontLeftTurningMotorPort = DRIVE.motors.turning.frontLeft;
        public static final int frontRightTurningMotorPort = DRIVE.motors.turning.frontRight;
        public static final int backLeftTurningMotorPort = DRIVE.motors.turning.backLeft;
        public static final int backRightTurningMotorPort = DRIVE.motors.turning.backRight;

        public static final int frontLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontLeft;
        public static final int frontRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontRight;
        public static final int backLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backLeft;
        public static final int backRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backRight;

        public static final double frontLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontLeft;
        public static final double frontRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontRight;
        public static final double backLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backLeft;
        public static final double backRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backRight;

        public static final boolean frontLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.frontLeft;
        public static final boolean frontRightDriveEncoderReversed = DRIVE.encoders.driveReversed.frontRight;
        public static final boolean backLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.backLeft;
        public static final boolean backRightDriveEncoderReversed = DRIVE.encoders.driveReversed.backRight;

        public static final boolean frontLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.frontLeft;
        public static final boolean frontRightTurningEncoderReversed = DRIVE.encoders.turningReversed.frontRight;
        public static final boolean backLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.backLeft;
        public static final boolean backRightTurningEncoderReversed = DRIVE.encoders.turningReversed.backRight;

        public static final boolean frontLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontLeft;
        public static final boolean frontRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontRight;
        public static final boolean backLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backLeft;
        public static final boolean backRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backRight;

        public static final double physicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;
        public static final double physicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;
        public static final double teleDriveMaxSpeedMetersPerSecond =physicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;
        public static final double teleDriveMaxAngularSpeedRadiansPerSecond = physicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;

        private DriveConstants() {
        }
    }

    /** Operator-interface constants such as ports and button mappings. */
    public static final class OIConstants {
        public static final int driverControllerPort = DRIVE.oi.driverControllerPort;
        public static final int driverYAxis = DRIVE.oi.driverYAxis;
        public static final int driverXAxis = DRIVE.oi.driverXAxis;
        public static final int driverRotAxis = DRIVE.oi.driverRotAxis;
        public static final int shooterToggleButtonIdx = DRIVE.oi.shooterToggleButtonIdx;
        public static final int indexerToggleButtonIdx = DRIVE.oi.indexerToggleButtonIdx;
        public static final int driverLeftTriggerAxis = DRIVE.oi.driverLeftTriggerAxis;
        public static final int driverRightTriggerAxis = DRIVE.oi.driverRightTriggerAxis;
        public static final int driverFieldOrientedButtonIdx = DRIVE.oi.driverFieldOrientedButtonIdx;
        public static final int driveUnlockSwerveButtonIdx = DRIVE.oi.driveUnlockSwerveButtonIdx;
        public static final int aimAtTagButtonIdx = DRIVE.oi.aimAtTagButtonIdx;
        public static final int calibrateAbsoluteEncoderButtonIdx = DRIVE.oi.calibrateAbsoluteEncoderButtonIdx;
        public static final int aprilTagAlignButtonIdx = DRIVE.oi.aprilTagAlignButtonIdx;
        public static final double triggerPressedThreshold = DRIVE.oi.triggerPressedThreshold;
        public static final double deadband = DRIVE.oi.deadband;

        private OIConstants() {
        }
    }

    /** Camera mount and Limelight identity constants. */
    public static final class LimelightConstants {
        public static final String limelightName = VISION.limelight.name;
        public static final double lensHeightMeters = Units.inchesToMeters(VISION.limelight.lensHeightInches);
        public static final double mountAngleDegrees = VISION.limelight.mountAngleDegrees;
        public static final double defaultTagHeightMeters = Units.inchesToMeters(VISION.limelight.defaultTagHeightInches);

        private LimelightConstants() {
        }
    }

    /** Vision behavior and target-geometry constants. */
    public static final class VisionConstants {
        public static final double aimKp = VISION.aim.kp;
        public static final double aimMaxAngularSpeedRadPerSec = VISION.aim.maxAngularSpeedRadPerSec;
        public static final double aimToleranceDegrees = VISION.aim.toleranceDegrees;
        public static final double hubCenterHeightMeters = VISION.hub.centerHeightMeters;
        public static final double hubOpeningWidthMeters = VISION.hub.openingWidthMeters;
        public static final double hubOpeningHeightMeters = VISION.hub.openingHeightMeters;
        public static final double hubFrontEdgeToCenterMeters = VISION.hub.frontEdgeToCenterMeters;
        public static final double hubAprilTagLateralOffsetMeters = VISION.hub.aprilTagLateralOffsetMeters;

        private VisionConstants() {
        }
    }

    /** Constants used by the shooter, hood, indexer, and shot solver. */
    public static final class ShooterConstants {
        public static final int shooterMotorId = SHOOTER.motors.flywheel.leader;
        public static final boolean shooterMotorInverted = SHOOTER.motors.flywheel.leaderInverted;
        public static final int indexerMotorId = SHOOTER.motors.indexer.system;
        public static final boolean indexerMotorInverted = SHOOTER.motors.indexer.inverted;
        public static final double shooterFullSpeed = SHOOTER.flywheel.fullSpeed;
        public static final double shooterLaunchVelocityMetersPerSecond = SHOOTER.flywheel.launchVelocityMetersPerSecond;
        public static final double indexerFullSpeed = SHOOTER.indexer.fullSpeed;
        public static final double hoodInitialAngleRadians = Math.toRadians(SHOOTER.hood.initialAngleDegrees);
        public static final double hoodMaxAngleRadians = Math.toRadians(SHOOTER.hood.maxAngleDegrees);
        public static final double projectileReleaseHeightMeters = SHOOTER.projectile.releaseHeightMeters;
        public static final double projectileBallMassKg = SHOOTER.projectile.ballMassKg;
        public static final double projectileBallDiameterMeters = SHOOTER.projectile.ballDiameterMeters;
        public static final double projectileDragCoefficient = SHOOTER.projectile.dragCoefficient;
        public static final double projectileAirDensityKgPerCubicMeter = SHOOTER.projectile.airDensityKgPerCubicMeter;
        public static final double projectileFlywheelBallFrictionCoefficient = SHOOTER.projectile.flywheelBallFrictionCoefficient;
        public static final double projectileGravityMetersPerSecondSquared = SHOOTER.projectile.gravityMetersPerSecondSquared;
        public static final double projectileSolverTimeStepSeconds = SHOOTER.projectile.solverTimeStepSeconds;
        public static final double projectileSolverMaxTimeSeconds = SHOOTER.projectile.solverMaxTimeSeconds;
        public static final double projectileControlLatencySeconds = SHOOTER.projectile.controlLatencySeconds;

        private ShooterConstants() {
        }
    }
}
