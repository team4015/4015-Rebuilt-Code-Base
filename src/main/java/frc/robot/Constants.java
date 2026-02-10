package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);
    
    public static final double kDriveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;

    public static final double kTurningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;

    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;

    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

    public static final double kPTurning = DRIVE.module.turningKP; 
  }

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(DRIVE.dimensions.trackWidthInches);

    public static final double kWheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2)
        );

    public static final int kFrontLeftDriveMotorPort = DRIVE.motors.drive.frontLeft;
    public static final int kFrontRightDriveMotorPort = DRIVE.motors.drive.frontRight;
    public static final int kBackLeftDriveMotorPort = DRIVE.motors.drive.backLeft;
    public static final int kBackRightDriveMotorPort = DRIVE.motors.drive.backRight;

    public static final int kFrontLeftTurningMotorPort = DRIVE.motors.turning.frontLeft;
    public static final int kFrontRightTurningMotorPort = DRIVE.motors.turning.frontRight;
    public static final int kBackLeftTurningMotorPort = DRIVE.motors.turning.backLeft;
    public static final int kBackRightTurningMotorPort = DRIVE.motors.turning.backRight;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontLeft;
    public static final int kFrontRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontRight;
    public static final int kBackLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backLeft;
    public static final int kBackRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backRight;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontLeft;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontRight;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backLeft;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backRight;

    public static final boolean kFrontLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.frontLeft;
    public static final boolean kFrontRightDriveEncoderReversed = DRIVE.encoders.driveReversed.frontRight;
    public static final boolean kBackLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.backLeft;
    public static final boolean kBackRightDriveEncoderReversed = DRIVE.encoders.driveReversed.backRight;

    public static final boolean kFrontLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.frontLeft;
    public static final boolean kFrontRightTurningEncoderReversed = DRIVE.encoders.turningReversed.frontRight;
    public static final boolean kBackLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.backLeft;
    public static final boolean kBackRightTurningEncoderReversed = DRIVE.encoders.turningReversed.backRight;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontLeft;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontRight;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backLeft;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backRight;

    // Speed limits
    public static final double kPhysicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;
  }
  

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;
  }
}
