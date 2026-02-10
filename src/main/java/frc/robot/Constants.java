package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
  
  public static final class ModuleConstants {
    public static final double wheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);
    
    public static final double driveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;

    public static final double turningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;

    public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;

    public static final double turningEncoderRot2Rad = turningMotorGearRatio * 2.0 * Math.PI;

    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60.0;

    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad / 60.0;

    public static final double kPTurning = DRIVE.module.turningKP; 
  }

  public static final class DriveConstants {

    public static final double trackWidth = Units.inchesToMeters(DRIVE.dimensions.trackWidthInches);

    public static final double wheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d( wheelBase / 2, -trackWidth / 2),
            new Translation2d( wheelBase / 2,  trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2,  trackWidth / 2)
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

    // Speed limits
    public static final double physicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;

    public static final double physicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;

    public static final double teleDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;

    public static final double teleDriveMaxAngularSpeedRadiansPerSecond = physicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;
  }
  

  public static final class OIConstants {
    public static final int driverControllerPort = 0;

    public static final int driverYAxis = 1;
    public static final int driverXAxis = 0;
    public static final int driverRotAxis = 4;
    public static final int driverFieldOrientedButtonIdx = 1;

    public static final double deadband = 0.05;
  }
}
