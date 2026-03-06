package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Robot-wide constants derived from static values and deploy-time JSON config.
 */
public final class Constants {
    /** Loaded drivetrain configuration from {@code src/main/deploy/driveConfig.json}. */
    public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();

    private Constants() {
    }

    /**
     * Constants specific to module mechanics, encoder conversions, and steering control gain.
     */
    public static final class ModuleConstants {
        /** Wheel diameter in meters. */
        public static final double wheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);

        /** Reciprocal drive reduction used to convert motor rotations to wheel rotations. */
        public static final double driveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;
        /** Reciprocal steering reduction used to convert motor rotations to module angle rotations. */
        public static final double turningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;

        /** Drive encoder position conversion (motor rotations -> wheel meters). */
        public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;
        /** Turning encoder position conversion (motor rotations -> radians). */
        public static final double turningEncoderRot2Rad = turningMotorGearRatio * 2.0 * Math.PI;

        /** Drive encoder velocity conversion (motor RPM -> wheel m/s). */
        public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60.0;
        /** Turning encoder velocity conversion (motor RPM -> rad/s). */
        public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad / 60.0;

        /** Proportional steering gain for module angle controller. */
        public static final double kPTurning = DRIVE.module.turningKP;

        private ModuleConstants() {
        }
    }

    /**
     * Drivetrain dimensions, IDs, inversion flags, and speed limits.
     */
    public static final class DriveConstants {
        /** Track width in meters (left-to-right wheel center distance). */
        public static final double trackWidth = Units.inchesToMeters(DRIVE.dimensions.trackWidthInches);
        /** Wheel base in meters (front-to-back wheel center distance). */
        public static final double wheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

        /** Swerve kinematics model based on module corner positions relative to robot center. */
        public static final SwerveDriveKinematics driveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );

        /** Front-left drive motor CAN ID. */
        public static final int frontLeftDriveMotorPort = DRIVE.motors.drive.frontLeft;
        /** Front-right drive motor CAN ID. */
        public static final int frontRightDriveMotorPort = DRIVE.motors.drive.frontRight;
        /** Back-left drive motor CAN ID. */
        public static final int backLeftDriveMotorPort = DRIVE.motors.drive.backLeft;
        /** Back-right drive motor CAN ID. */
        public static final int backRightDriveMotorPort = DRIVE.motors.drive.backRight;

        /** Front-left turning motor CAN ID. */
        public static final int frontLeftTurningMotorPort = DRIVE.motors.turning.frontLeft;
        /** Front-right turning motor CAN ID. */
        public static final int frontRightTurningMotorPort = DRIVE.motors.turning.frontRight;
        /** Back-left turning motor CAN ID. */
        public static final int backLeftTurningMotorPort = DRIVE.motors.turning.backLeft;
        /** Back-right turning motor CAN ID. */
        public static final int backRightTurningMotorPort = DRIVE.motors.turning.backRight;

        /** Front-left absolute encoder analog channel. */
        public static final int frontLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontLeft;
        /** Front-right absolute encoder analog channel. */
        public static final int frontRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontRight;
        /** Back-left absolute encoder analog channel. */
        public static final int backLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backLeft;
        /** Back-right absolute encoder analog channel. */
        public static final int backRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backRight;

        /** Front-left absolute steering offset in radians. */
        public static final double frontLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontLeft;
        /** Front-right absolute steering offset in radians. */
        public static final double frontRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontRight;
        /** Back-left absolute steering offset in radians. */
        public static final double backLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backLeft;
        /** Back-right absolute steering offset in radians. */
        public static final double backRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backRight;

        /** Front-left drive encoder inversion flag. */
        public static final boolean frontLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.frontLeft;
        /** Front-right drive encoder inversion flag. */
        public static final boolean frontRightDriveEncoderReversed = DRIVE.encoders.driveReversed.frontRight;
        /** Back-left drive encoder inversion flag. */
        public static final boolean backLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.backLeft;
        /** Back-right drive encoder inversion flag. */
        public static final boolean backRightDriveEncoderReversed = DRIVE.encoders.driveReversed.backRight;

        /** Front-left turning encoder inversion flag. */
        public static final boolean frontLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.frontLeft;
        /** Front-right turning encoder inversion flag. */
        public static final boolean frontRightTurningEncoderReversed = DRIVE.encoders.turningReversed.frontRight;
        /** Back-left turning encoder inversion flag. */
        public static final boolean backLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.backLeft;
        /** Back-right turning encoder inversion flag. */
        public static final boolean backRightTurningEncoderReversed = DRIVE.encoders.turningReversed.backRight;

        /** Front-left absolute encoder reversal flag. */
        public static final boolean frontLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontLeft;
        /** Front-right absolute encoder reversal flag. */
        public static final boolean frontRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontRight;
        /** Back-left absolute encoder reversal flag. */
        public static final boolean backLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backLeft;
        /** Back-right absolute encoder reversal flag. */
        public static final boolean backRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backRight;

        /** Physical drivetrain translation speed limit (m/s). */
        public static final double physicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;
        /** Physical drivetrain angular speed limit (rad/s). */
        public static final double physicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;

        /** Teleop translation speed limit (m/s), scaled from physical max. */
        public static final double teleDriveMaxSpeedMetersPerSecond =
            physicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;
        /** Teleop angular speed limit (rad/s), scaled from physical max. */
        public static final double teleDriveMaxAngularSpeedRadiansPerSecond =
            physicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;

        private DriveConstants() {
        }
    }

    /**
     * Operator-input constants for joystick mapping and deadband.
     */
    public static final class OIConstants {
        /** USB port index for the primary driver controller. */
        public static final int driverControllerPort = 0;

        /** Axis index for forward/backward command. */
        public static final int driverYAxis = 1;
        /** Axis index for left/right strafe command. */
        public static final int driverXAxis = 0;
        /** Axis index for rotational command. */
        public static final int driverRotAxis = 4;
        /** Button index used to enable field-oriented driving while held. */
        public static final int driverFieldOrientedButtonIdx = 1;
        /** Button index used to temporarily unlock/coast swerve modules while held. */
        public static final int driveUnlockSwerveButtonIdx = 2;

        /** Joystick deadband used to suppress small stick noise/drift. */
        public static final double deadband = 0.05;

        private OIConstants() {
        }
    }
}
