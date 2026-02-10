package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
  
  public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);
    
    public static final double kDriveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;

    public static final double kTurningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;

    public static final double kTurningEncoderRot2Rad = kDriveMotorGearRatio * 2.0 * Math.PI;

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;

     public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

    public static final double kPTurning = DRIVE.module.turningKP; 
  }

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(DRIVE.dimensions.trachWidthInches);

    public static final double kWheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2)
        );

    // Speed limits
    public static final double kPhysicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;
  }
  

  public static final class OIConstants{
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;
  }
}
